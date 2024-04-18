import time
import sensor
from pyb import UART
from machine import I2C
from vl53l1x import VL53L1X
import image
import math
from machine import Pin
import sensor, image
from pyb import LED
import time
import math
import omv


""" MACROS for balloon detection """
# Grid setup
N_ROWS = 10
N_COLS = 15

# Probablistic filter for detection
FILTER = True
L_MAX = 15
L_MIN = -8

# print the stats at the upper left corner
PRINT_CORNER = False

# whether combine detction of green and purple as target balloons
COMBINE_GREEN_AND_PURPLE = True

# whether remove blue color from purple color
SEPARATE_BLUE_AND_PURPLE = True

# manual white balance - to be used with *get_gains.py* in the repository
# - see RGB gain readings in the console
R_GAIN, G_GAIN, B_GAIN = 72, 62, 105

# reference line segments for different colors of balloons
COLOR_LINE_REF_PURPLE = [[26, -40], [11, -20]]
COLOR_LINE_REF_GREEN = [[-15, 13], [-19, 17]]
COLOR_LINE_REF_BLUE = [[20, -46], [1, -4]]
# TODO: replace with the following parameters

# Color distribution
COLOR_PURPLE_MEAN = [ 14.85185185, -32.2962963 ]
COLOR_PURPLE_INV_COV = [[0.36527085581188184, 0.2904622446008661], [0.29046224460086617, 0.27376389388412165]]
DECAY_FACTOR = 4.0

# maximum distance from the stats of a grid to a line reference to be considered the color
MAX_DIST_PURPLE = 8.0
MAX_DIST_GREEN = 8.0
MAX_DIST_BLUE = 3.0

# allowed standard deviation range for a color detection
# lower bound filters out uniform colors such as a light source
# higher bound filters out messy background/environment
STD_RANGE_PURPLE = [5, 30]
STD_RANGE_GREEN = [5, 30]
STD_RANGE_BLUE = [5, 30]

# a semi-adaptive auto exposure for environment with high contrast
BACKLIGHT_TOO_MUCH = False

CAP_EXP_COUNTER = 100
AES_EVERY = 50
BRIGHTNESS_MULT = 1.6

""" MACROS for goal detection """
NORM_LEVEL = 2  # Default to use L2 norm, change to L1 to reduce computation
MAX_FEATURE_DIST = 32767  # The maximum feature distance
FF_POSITION = 0.0 # The forgetting factor for the position
FF_SIZE = 0.0 # The forgetting factor for the size
GF_POSITION = 0.3 # The gain factor for the position
GF_SIZE = 0.3 # The gain factor for the size
FRAME_PARAMS = [0, 0, 240, 160] # Upper left corner x, y, width, height

frame_rate = 80 # target framerate that is a lie
ORANGE_TARGET = [(55, 100, -12, 13, 27, 54)]
TARGET_COLOR = [(54, 89, -38, 0, 16, 49)]#(48, 100, -44, -14, 30, 61)]#[(54, 100, -56, -5, 11, 70), (0, 100, -78, -19, 23, 61)]#[(49, 97, -45, -6, -16, 60),(39, 56, -12, 15, 48, 63), (39, 61, -19, 1, 45, 64), (20, 61, -34, 57, -25, 57)] # orange, green
WAIT_TIME_US = 1000000//frame_rate

SATURATION = 128 # global saturation for goal detection mode - not affected by ADVANCED_SENSOR_SETUP, defeult 64
CONTRAST = 40 # global contrast for goal detection mode - not affected by ADVANCED_SENSOR_SETUP, defeult 48
ADVANCED_SENSOR_SETUP = False # fine-tune the sensor for goal


""" Balloon grid detection classes and functions """
# color confidence filter
class LogOddFilter:
    def __init__(self, n, p_det=0.95, p_ndet=0.1, p_x=0.5):
        """

        p_x: Probability of having a balloon in a cell

        """

        self.init_belif = math.log(p_x/(1-p_x))
        self.l_det = math.log(p_det/(1-p_det))
        self.l_ndet = math.log(p_ndet / (1 - p_ndet))
        self.L = [0. for _ in range(n)]
        self.P = [0. for _ in range(n)]
        print("Initial belief=", self.init_belif, "L for detection=", self.l_det, "L for not detection", self.l_ndet )


    def update(self, measurements, l_max=L_MAX, l_min=L_MIN):
        for i, z in enumerate(measurements):
            # Detected or not detected
            li = self.l_det if z else self.l_ndet
            # Belief for li
            self.L[i]+= li -self.init_belif
            # Cap
            self.L[i] = min(l_max, max(l_min, self.L[i]))


        return self.L

    def probabilities(self):
        for i, l in enumerate(self.L):
                self.P[i] = 1. / (1. + math.exp(-l))
        return self.P

# color detector based on distance to a line segment on L(AB) color space
class ColorDetector:
    def __init__(self, color_id, line_ref, max_dist,
                 std_range, rgb, mahalanobis, mu=None,
                 sigma_inv=None, decay = DECAY_FACTOR):
        self.color_id = color_id
        self.line_ref = line_ref
        self.max_dist = max_dist
        self.std_range = std_range
        self.rgb = rgb
        # Color distribution
        self.mahalanobis = mahalanobis
        if mahalanobis:
            self.mu = mu  # mu (list): Mean vector.
            self.sigma_inv = sigma_inv  # sigma_inv (list of lists): Inverse covariance matrix.
            self.determinant = 1 / (sigma_inv[0][0] * sigma_inv[1][1] - sigma_inv[1][0] * sigma_inv[1][0])
            print("det ",self.determinant)
            self.decay = decay

        self.metric = [0. for _ in range(N_COLS*N_ROWS)]

        # Filter
        self.filter = LogOddFilter(N_ROWS * N_COLS)
        self.P = [0. for _ in range(N_COLS*N_ROWS)]


    def _distance(self, point, std):
        (l, a, b) = point
        if self.mahalanobis:
            d = self._distance_mahalanobis((a, b))
            # print(d, end=", ")
            # Clamp max distance
            max_num_std = 5
            d = d if d < max_num_std else max_num_std

            d = 1 - d / max_num_std
        else:
            d = self._distance_point_to_segment((a, b), self.line_ref)

            # Clamp max distance
            d = d if d < self.max_dist else self.max_dist

            # Distance is 1 if close to the line, and 0 if outside the line
            d = 1 - d / self.max_dist

        # Check if standard deviation is in range
        for s in std:
            if not self.std_range[0] < s < self.std_range[1]:
                d = 0.0
                break

        # Too much lightening
        if not 10 < l < 60:  #fixme magic numbers
            d = 0.0

        return d


    def _distance_mahalanobis(self, point):
        """
        Compute the Mahalanobis distance between a point and a distribution. It uses
        Parameters:
            point (list): Point vector.
        Returns:
            float: Mahalanobis distance.
        """
        # Calculate the difference between the point and the mean
        diff = [x - y for x, y in zip(point, self.mu)]

        # Compute the Mahalanobis distance
        mahalanobis_sq = self._dot_product(diff, [self._dot_product(sigma_inv_row, diff) for sigma_inv_row in self.sigma_inv])
        mahalanobis_dist = mahalanobis_sq ** 0.5
        return mahalanobis_dist

    def _dot_product(self, a, b):
        """Compute the dot product of two vectors."""
        return sum(x * y for x, y in zip(a, b))

    def update_cell(self, row, col, point, std):
        d = self._distance(point, std)
        self.metric[row * N_COLS + col] = d
        return d

    def update_filter(self):
        self.filter.update(self.metric)
        self.P = self.filter.probabilities()
        return self.P

    def _distance_point_to_segment(self, point, segment):
        x1, y1 = segment[0]
        x2, y2 = segment[1]
        x0, y0 = point

        dx = x2 - x1
        dy = y2 - y1

        # Compute the dot product of the vector from the first endpoint of the segment to the point
        # with the vector of the segment
        dot_product = ((x0 - x1) * dx + (y0 - y1) * dy) / (dx * dx + dy * dy)

        # Clamp the dot product to ensure the closest point lies within the line segment
        t = max(0, min(1, dot_product))

        # Calculate the coordinates of the closest point on the line segment
        closest_point = (x1 + t * dx, y1 + t * dy)

        # Calculate the distance between the given point and the closest point
        distance = math.sqrt((x0 - closest_point[0]) ** 2 + (y0 - closest_point[1]) ** 2)

        return distance


def _normal_dist(mu=0, sigma=1):
    u1 = random.uniform(0, 1)
    u2 = random.uniform(0, 1)
    return mu + sigma*math.sqrt(-2.0 * math.log(u1)) * math.cos(2 * math.pi * u2)

# grid-based color detector divisions
class Grid:
    def __init__(self, num_rows, num_cols, img_width, img_height):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.cell_width = int(img_width / num_cols)
        self.cell_height = int(img_height / num_rows)

    def count(self, img, detectors):
        # Loop through each cell of the grid
        for row in range(self.num_rows):
            for col in range(self.num_cols):
                # Define the region of interest (ROI) for the current cell
                roi = (col * self.cell_width, row * self.cell_height, self.cell_width, self.cell_height)
                img_roi = img.copy(roi=roi)

                # Calculate the mean and variance of the ROI
                s = img_roi.get_statistics()

                for detector in detectors:
                    d_mean = detector.update_cell(row, col, (s.l_mean(), s.a_mean(), s.b_mean()),
                                                (s.l_stdev(), s.a_stdev(), s.b_stdev()))

                if PRINT_CORNER and row<3 and col<3:
                    print((s.a_mean(), s.b_mean()), end=', ')

            if PRINT_CORNER:
                print()



    def plot_metric(self, metrics, rgb):

        for row in range(self.num_rows):
            for col in range(self.num_cols):
                roi = (col * self.cell_width, row * self.cell_height, self.cell_width, self.cell_height)

                metric = metrics[row*self.num_cols + col]

                # Draw the number of ones in the corner of the cell
#                img.draw_string(roi[0], roi[1],str(int(metric*10)) , color=(0,255,0))  #

                if metric > 0.001 or (PRINT_CORNER and row<3 and col<3):
                    # Draw the ROI on the image
                    img.draw_rectangle(roi, color=(int(metric*rgb[0]),int(metric*rgb[1]),int(metric*rgb[2])), thickness=1)

    def normalize(self, ones):
        totalones = max(sum(ones), 1)
        nones = [i / totalones for i in ones]
        return nones

    def statistics(self, nones):
        # Compute the mean of the sample vector
        mean = sum(nones) / len(nones)
        # Compute the sum of squared differences from the mean
        squared_diff_sum = sum((x - mean) ** 2 for x in nones)
        # Compute the variance
        variance = squared_diff_sum / len(nones)

        return mean,variance


    def count_column(self, nones, col):
        col_sum = sum([nones[col + i * self.num_cols] for i in range(self.num_rows)])
        return col_sum

    def count_row(self, nones, row):
        row_sum = sum([nones[i + self.num_cols * row] for i in range(self.num_cols)])
        return row_sum

    def action_differential(self, metric):

        totalm= sum(metric) if sum(metric)>0 else 1
        metric = [m / totalm for m in metric]

        # Counting
        left_cells = sum([self.count_column(metric, i) for i in range(N_COLS//2) ])
        right_cells = sum([self.count_column(metric, N_COLS-i-1) for i in range(N_COLS//2) ])
        up_cells =  sum([self.count_row(metric, i) for i in range(N_ROWS//2) ])
        down_cells = sum([self.count_row(metric, N_ROWS-i-1) for i in range(N_ROWS//2) ])


        ux, uy = 0,0

        # Control action
        # print(left_cells, right_cells, up_cells, down_cells)
        ux = int((right_cells - left_cells) * img.width()) // N_COLS
        uy = -int((up_cells - down_cells) * img.height()) // N_ROWS

        return ux, uy

    def _index_to_matrix(self, i):
        row = i // self.num_cols
        col = i % self.num_cols
        return row, col

    def _matrix_to_index(self, row, col):
        index = row * self.num_cols + col
        return index

    def action(self, metric):
        max_col = -1
        max_row = -1
        max_val = -1
        for i, m in enumerate(metric):
            row, col = self._index_to_matrix(i)

            # Out of bounds
            if row==0 or col==0 or row==self.num_rows-1 or col== self.num_cols-1:
                continue

            total=m
            # Neighbors:
            for k1 in range(3):
                for k2 in range(3):
                    r, c = row-1+k1, col-1+k2  # Row and column
                    mk = self._matrix_to_index(r, c)
                    total += metric[mk]

            if total>max_val:
                max_col = col
                max_row = row
                max_val = total




        x, y = max_col * self.cell_width + self.cell_width// 2, max_row * self.cell_height + self.cell_height // 2
        radius = int(30 * max_val/9)
        # Draw the ROI on the image

        if max_val > 0.05:
            img.draw_circle(x, y, radius, color=(255,0,0), thickness=4, fill=False)


        return x, y, max_val

    def weighted_average(self, metrics):
        cell_row = self.num_rows/2.0 - 0.5
        cell_col = self.num_cols/2.0 - 0.5

        sum_score = sum(metrics)
        if sum_score > 0:
            led_blue.on()
            led_red.on()
            cell_row = 0.0
            cell_col = 0.0
            for row in range(self.num_rows):
                for col in range(self.num_cols):
                    index = row*self.num_cols + col
                    cell_row += (metrics[index] * row)
                    cell_col += (metrics[index] * col)

            cell_row /= sum_score
            cell_col /= sum_score
        else:
            led_blue.off()
            led_red.off()
        return cell_row + 0.5, cell_col + 0.5, sum_score

# Dummy class for the balloon detection to use with the tracker in the main function
class BalloonTracker:
    # TODO: this is NOT a child class of Tracker
    def __init__(self, grid, detectors,
                 filter_on=FILTER,
                 separate_blue_purple=SEPARATE_BLUE_AND_PURPLE,
                 combine_green_purple=COMBINE_GREEN_AND_PURPLE):
        self.grid = grid
        self.detectors = detectors
        self.filter_on = filter_on
        self.separate_blue_purple = separate_blue_purple
        self.combine_green_purple = combine_green_purple
        self.flag = int(0)
        self.led_red = LED(1)
        self.led_green = LED(2)
        self.led_blue = LED(3)

    def track(self, img):
        self.grid.count(img, detectors)

        for detector in self.detectors:
            metric_grid = detector.metric

            if self.filter_on:
               metric_grid = detector.update_filter()

            grid.plot_metric(metric_grid, detector.rgb)

        # Discard purple if blue has higher probability
        if self.separate_blue_purple:
            new_purple = [p if p>b else 0 for p,b in zip(purpleDet.P, blueDet.P)]

        # Combine green and purple
        if self.combine_green_purple:
            metric_grid = [max(p,g) for p,g in zip(new_purple, greenDet.P)]

        total_score = max(metric_grid)
        ux, uy, val = grid.action(metric_grid)

        # LED indicator and flag toggle
        if total_score > 0.05:
            if max(new_purple) > max(greenDet.P):
                self.led_red.on()
                self.led_blue.on()
                self.led_green.off()
            elif max(new_purple) < max(greenDet.P):
                self.led_red.off()
                self.led_blue.off()
                self.led_green.on()
            else:
                self.led_red.on()
                self.led_blue.on()
                self.led_green.on()
            if self.flag:
                self.flag = 3 - self.flag
            else:
                self.flag = 1
        else:
            self.led_red.off()
            self.led_blue.off()
            self.led_green.off()
            self.flag = 0

        x_roi, y_roi = ux, uy
        w_roi, h_roi = int(10*val), int(10*val)
        x_value, y_value = ux, uy
        return [x_roi, y_roi, w_roi, h_roi, x_value, y_value, w_roi, h_roi, 0.0], self.flag | 0x40

""" Goal detection classes and functions """
# a moving ROI that shrinks towards a current detection and expands to the framesize without an active detection
class MemROI:
    def __init__(self, frame_params:list = FRAME_PARAMS,
                 min_windowsize:int=20, ffp:float=FF_POSITION, ffs:float=FF_SIZE,
                 gfp:float=GF_POSITION, gfs:float=GF_SIZE)->None:
        """
        @description: Constructor of the ROI object that memorizes previous states.
        @param       {*} self:
        @param       {list} frame_params: The parameters of the frame [x0, y0, max_w, max_h]
        @param       {int} min_windowsize: The minimum size of the tracking window
        @param       {float} ffp: The forgetting factor for the position
        @param       {float} ffs: The forgetting factor for the size
        @param       {float} gfp: The gain factor for the position
        @param       {float} gfs: The gain factor for the size
        @return      {*} None
        """
        print(frame_params)
        self.roi = frame_params # [x0, y0, w, h]
        self.frame_params = frame_params  # [x0, y0, max_w, max_h]
        self.min_windowsize = min_windowsize
        self.ffp = ffp
        self.ffs = ffs
        self.gfp = gfp
        self.gfs = gfs

    def _clamp(self)->None:
        """
        @description: Clamp the ROI to be within the frame.
        @param       {*} self:
        @return      {*} None
        """
        # Ensure the ROI's top-left corner is within the bounds.
        self.roi[0] = max(self.frame_params[0], self.roi[0])
        self.roi[1] = max(self.frame_params[1], self.roi[1])

        # Ensure the ROI's bottom-right corner is within the bounds.
        self.roi[2] = min(self.frame_params[2] - self.roi[0], self.roi[2])
        self.roi[3] = min(self.frame_params[3] - self.roi[1], self.roi[3])

    def _center(self, rect:list)->tuple:
        """
        @description: Calculate the center of the rectangle.
        @param       {*} self: -
        @param       {list} rect: The rectangle to be calculated [Upper left corner x, y, w, h]
        @return      {tuple} The center of the rectangle
        """
        if len(rect) != 4:
            raise ValueError("Cannot calculate the center of the rectangle! The rectangle must be in the form of [x0, y0, w, h]")
        return (rect[0] + rect[2] / 2, rect[1] + rect[3] / 2)

    def _map(self, rect1:list, rect2:list, flag:int)->list:
        """
        @description: Map rect1 to rect2 by the forgetting factors.
        @param       {*} self:
        @param       {list} rect1: Rectangle to be mapped [x0, y0, w, h]
        @param       {list} rect2: Rectangle to be mapped to [x0, y0, w, h]
        @param       {int} flag: 0 for forgetting factor, 1 for gain factor
        @return      {list} The mapped rectangle [x0, y0, w, h]
        """
        # Get the centers of the rectangles
        cx1, cy1 = self._center(rect1) # Center x, y
        cx2, cy2 = self._center(rect2) # Center x, y

        fp = 0.0
        fs = 0.0
        if flag == 0:
            fp = self.ffp
            fs = self.ffs
        elif flag == 1:
            fp = self.gfp
            fs = self.gfs
        else:
            raise ValueError("Invalid factor setting! flag must be 0(forget) or 1(gain).")

        # Calculate new center by shifting rect1's center towards rect2's center by alpha
        new_cx = cx1 + fp * (cx2 - cx1)
        new_cy = cy1 + fp * (cy2 - cy1)

        # Shift the size of rect1 towards rect2's size by beta
        new_w = rect1[2] + fs * (rect2[2] - rect1[2])
        new_h = rect1[3] + fs * (rect2[3] - rect1[3])
        return [new_cx - new_w / 2, new_cy - new_h / 2, new_w, new_h]


    def update(self, new_roi:list=None)->None:
        """
        @description: Update the ROI with a new ROI.
        @param       {*} self:
        @param       {list} new_roi: The new roi to map to [x0, y0, w, h]
        @return      {*} None
        """
        if not new_roi: # No new detection is found in the maximum tracking window
            self.roi = self._map(self.roi, self.frame_params, 0) # Map the ROI to the frame by the forgetting factors
        else:
            # Scale up the new_roi
            expanded_roi = [new_roi[0] - 0.15 * new_roi[2],
                            new_roi[1] - 0.15 * new_roi[3],
                            1.3 * new_roi[2],
                            1.3 * new_roi[3]]

            self.roi = self._map(self.roi, expanded_roi, 1) # Map the ROI to the new_roi by the gain factors
        self._clamp() # Clamp the ROI to be within the frame

    def reset(self)->None:
        """
        @description: Reset the ROI to the frame.
        @param       {*} self:
        @return      {*} None
        """
        self.roi = self.frame_params

    def get_roi(self)->list:
        """
        @description: Get the ROI.
        @param       {*} self:
        @return      {list} The ROI [x0, y0, w, h]
        """
        return [math.ceil(value)+1 for value in self.roi]

# determine the shape of a detected goal
class ShapeDetector:
    def __init__(self, gridsize):
        self.gridsize = gridsize
        self.binary_image = sensor.alloc_extra_fb(gridsize, gridsize, sensor.BINARY)
        # Pre-create shapes
        self.img_triangle = self.create_triangle(gridsize)
        self.tot_tri = sum(self.img_triangle.get_pixel(j, i) for i in range(self.gridsize) for j in range(self.gridsize))
        self.img_circle = self.create_circle(gridsize)
        self.tot_cir = sum(self.img_circle.get_pixel(j, i) for i in range(self.gridsize) for j in range(self.gridsize))
        self.img_square = self.create_square(gridsize)
        self.tot_squ = sum(self.img_square.get_pixel(j, i) for i in range(self.gridsize) for j in range(self.gridsize))


    def __del__(self):
        sensor.dealloc_extra_fb()
        sensor.dealloc_extra_fb()
        sensor.dealloc_extra_fb()


    def create_triangle(self, gridsize):
        # Allocate frame buffer for triangle
    #        img = sensor.alloc_extra_fb(gridsize, gridsize, sensor.BINARY)
    #        # Draw an isosceles triangle
    #        img.draw_line(gridsize // 2, gridsize - 1, 0, 0, color=255, thickness=1)
    #        img.draw_line(gridsize // 2, gridsize - 1, gridsize - 1, 0, color=255, thickness=1)
    #        img.draw_line(0, 0, gridsize - 1, 0, color=255, thickness=2)
        img = sensor.alloc_extra_fb(gridsize, gridsize, sensor.BINARY)
        # Flipped isosceles triangle
        img.draw_line(gridsize // 2, 0, 0, gridsize - 1, color=255, thickness=1)  # Apex to left base
        img.draw_line(gridsize // 2, 0, gridsize - 1, gridsize - 1, color=255, thickness=1)  # Apex to right base
        img.draw_line(0, gridsize - 1, gridsize - 1, gridsize - 1, color=255, thickness=2)  # Base line

        return img


    def create_circle(self, gridsize):
        # Allocate frame buffer for circle
        img = sensor.alloc_extra_fb(gridsize, gridsize, sensor.BINARY)
        radius = (gridsize)// 2
        img.draw_circle(gridsize // 2, gridsize // 2, radius, color=255, fill=False, thickness=2)
        if (gridsize % 2 == 0):
            img.draw_circle((gridsize) // 2 -1, (gridsize) // 2 -1, radius, color=255, fill=False)
            img.draw_circle((gridsize) // 2 , (gridsize) // 2 -1, radius, color=255, fill=False)
            img.draw_circle((gridsize) // 2 -1, (gridsize) // 2 , radius, color=255, fill=False)
        return img

    def create_square(self, gridsize):
        # Allocate frame buffer for square
        img = sensor.alloc_extra_fb(gridsize, gridsize, sensor.BINARY)
        # Draw a square
        img.draw_rectangle(0, 0, gridsize, gridsize, color=255, fill=False, thickness=1)
        return img

    def downsample_and_average(self, roi_img):
        # Custom function to process and downsample the ROI
        # Use the mean_pooled function to simplify the pooling
        src_width, src_height = roi_img.width(), roi_img.height()
        block_width = src_width // self.gridsize
        block_height = src_height // self.gridsize
        width_remainder = src_width % self.gridsize
        height_remainder = src_height % self.gridsize
        if (block_width == 0 or block_height ==0):
            return self.binary_image

        # Iterate over each block
        for i in range(self.gridsize):
            for j in range(self.gridsize):
                current_block_width = block_width + (1 if j < width_remainder else 0)
                current_block_height = block_height + (1 if i < height_remainder else 0)
                x = sum(block_width + (1 if m < width_remainder else 0) for m in range(j))
                y = sum(block_height + (1 if n < height_remainder else 0) for n in range(i))

                # Define the sub ROI for this block
                sub_roi = (x, y, current_block_width, current_block_height)
                # Get statistics for the sub ROI
                stats = roi_img.get_statistics(roi=sub_roi)
                # Calculate the mean and determine if the block is predominantly white or black
                mean_val = stats.mean()
                binary_val = 1 if mean_val > 60 else 0  # Threshold the mean to create a binary image
                self.binary_image.set_pixel(j, i, binary_val)

        return self.binary_image

    def detect_shape(self, roi_img):
        mean_pooled_img = self.downsample_and_average(roi_img.to_grayscale())

        center_size = 3 if self.gridsize > 3 else 1  # Only 3x3 or 1x1, adjust if needed
        start = (self.gridsize - center_size) // 2
        end = start + center_size
        center_sum = sum(mean_pooled_img.get_pixel(j, i) for i in range(start, end) for j in range(start, end))
        if (center_sum >= center_size**2 * .66):
            return "not"
        # Prepare for shape comparison
        overlap_triangle = 0
        overlap_circle = 0
        overlap_square = 0

        # Calculate overlaps by comparing each pixel
        for i in range(self.gridsize):
            for j in range(self.gridsize):
                if mean_pooled_img.get_pixel(j, i) == 1:  # Check if the ROI pixel is white
                    if self.img_triangle.get_pixel(j, i) == 1:
                        overlap_triangle += 1
                    if self.img_circle.get_pixel(j, i) == 1:
                        overlap_circle += 1
                    if self.img_square.get_pixel(j, i) == 1:
                        overlap_square += 1

#        print("Overlap Triangle:", overlap_triangle, "Overlap Circle:", overlap_circle, "Overlap Square:", overlap_square)

        # Identify which shape it is based on maximum overlap
        if overlap_triangle > overlap_circle and overlap_triangle > overlap_square:
            return "triangle"
        elif overlap_square > overlap_circle:
            return "square"
        else:
            return "circle"

    def extract_valid_roi(self, img, blob, current_thresholds, min_edge_distance=0):
        """ Extracts and validates the ROI from the given blob based on minimum distance to the edge """
        left_distance = blob.x()
        right_distance = img.width() - (blob.x() + blob.w())
        top_distance = blob.y()
        bottom_distance = img.height() - (blob.y() + blob.h())
        min_distance = min(left_distance, right_distance, top_distance, bottom_distance)

        if min_distance >= min_edge_distance:
            roi_width = min(int(img.width() * 1), blob.w())
            roi_height = min(int(img.height() * 1), blob.h())
            if roi_width // self.gridsize > 0 and roi_height // self.gridsize > 0:
                roi = (max(0, blob.x()), max(0, blob.y()), roi_width, roi_height)
                x_scale = 1
                y_scale = 1
                if roi_width > img.width()/3:
                    y_scale = 1- roi_height/img.height()
                if roi_height > img.height()/3:
                    y_scale = 1- roi_height/img.height()
                try:
                    roi_img = img.copy(x_scale = x_scale, y_scale = y_scale, roi=roi).binary(current_thresholds)
                except:
                    return None, None
                return roi_img, roi

        return None, None  # Return None if no valid ROI found

# a moving blob object that memorizes previous blobs detected
class CurBLOB:
    def __init__(
        self,
        initial_blob,
        norm_level: int = NORM_LEVEL,
        feature_dist_threshold: int = 400,
        window_size=5,
        blob_id=0,
    ) -> None:
        """
        @description: Constructor of the blob object that memorizes previous states.
        @param       {*} self:
        @param       {*} initial_blob: The first blob appeared after the reset
        @param       {int} norm_level: The norm level for the feature distance (default to L2)
        @param       {int} feature_dist_threshold: The threshold for the feature distance (default to 100)
        @param       {*} window_size: The window size for the moving average (default to 3)
        @param       {*} blob_id: The id of the blob
        @return      {*} None
        """
        if initial_blob:
            self.blob_history = [initial_blob]
            self.feature_vector = [
                initial_blob.x(),
                initial_blob.y(),
                initial_blob.w(),
                initial_blob.h(),
                initial_blob.rotation_deg(),
            ]
        else:
            self.blob_history = None
            self.feature_vector = None


        self.norm_level = norm_level
        self.untracked_frames = 0  # number of frames that the blob is not tracked
        self.feature_dist_threshold = (
            feature_dist_threshold  # threshold for feature distance
        )
        self.window_size = window_size  # window size for moving average
        self.id = blob_id  # id of the blob

    def reset(self) -> None:
        """
        @description: Reset the current blob
        @param       {*} self:
        @return      {*} None
        """
        self.blob_history = None
        self.feature_vector = None
        self.untracked_frames = 0

    def reinit(self, blob: image.blob) -> None:
        """
        @description: Reinitialize the current blob with a new blob
        @param       {*} self:
        @param       {image.blob} blob: The new blob to be reinitialized with
        @return      {*} None
        """
        self.blob_history = [blob]  # reset the blob history
        self.feature_vector = [
            blob.x(),
            blob.y(),
            blob.w(),
            blob.h(),
            blob.rotation_deg(),
        ]
        self.untracked_frames = 0  # reset the untracked frames

    def compare(self, new_blob: image.blob) -> int:
        """
        @description: Compare the feature distance between the current blob and a new blob
        @param       {*} self:
        @param       {image.blob} new_blob: The new blob to be compared with
        @return      {int} The feature distance between the current blob and the new blob
        """
        new_feature = (
            new_blob.x(),
            new_blob.y(),
            new_blob.w(),
            new_blob.h(),
            new_blob.rotation_deg(),
        )  # get the feature vector of the new blob
        old_feature = self.feature_vector  # get the feature vector of the current blob
        if (
            not new_blob.code() == self.blob_history[-1].code()
        ):  # Check if the color is the same
            return MAX_FEATURE_DIST  # Different colors automatically grant a maximum distance
        elif self.norm_level == 1:  # The norm level is L1
            return sum([abs(new_feature[i] - old_feature[i]) for i in range(5)])
        elif self.norm_level == 2:  # The norm level is L2
            return math.sqrt(
                sum([(new_feature[i] - old_feature[i]) ** 2 for i in range(5)])
            )

    def update(self, list_of_blob: list) -> list:
        """
        @description: Update the current blob with the best candidate blob in the list of blobs
        @param       {*} self:
        @param       {list} list_of_blob: The list of blobs to be compared with
        @return      {list} The rectangle of the best candidate blob
        """
        if list_of_blob is None:  # For the case that no blob is detected
            self.untracked_frames += 1
            return None

        min_dist = 32767
        candidate_blob = None
        # Find the blob with minimum feature distance
        for b in list_of_blob:  # This should reference the input parameter 'list_of_blob', not 'blobs'
            dist = self.compare(b)
            if dist < min_dist:
                min_dist = dist
                candidate_blob = b

        if min_dist < self.feature_dist_threshold:
            # Update the feature history if the feature distance is below the threshold
            self.untracked_frames = 0  # Reset the number of untracked frames
            history_size = len(
                self.blob_history
            )  # Get the number of blobs in the history
            self.blob_history.append(candidate_blob)
            # Calculate the feature vector of the candidate blob
            candidate_feature = (
                candidate_blob.x(),
                candidate_blob.y(),
                candidate_blob.w(),
                candidate_blob.h(),
                candidate_blob.rotation_deg(),
            )

            if history_size < self.window_size:
                # Calculate the moving average directly if the blob history is not filled
                for i in range(5):
                    # calculate the moving average
                    self.feature_vector[i] = (self.feature_vector[i]*history_size +
                        candidate_feature[i])/(history_size + 1)
            else:
                # Remove the oldest blob from the history and calculate the moving average
                oldest_blob = self.blob_history[0]
                oldest_feature = (
                    oldest_blob.x(),
                    oldest_blob.y(),
                    oldest_blob.w(),
                    oldest_blob.h(),
                    oldest_blob.rotation_deg(),
                )
                self.feature_vector = [
                    (current * self.window_size - old + new) / history_size
                    for current, old, new in zip(
                        self.feature_vector, oldest_feature, candidate_feature
                    )
                ]
                self.blob_history.pop(0)
            return candidate_blob.rect()
        else:
            # Increase the number of untracked frames if no good candidate is found
            self.untracked_frames += 1
            return None

# base class for a tracking bounding box
class Tracker:
    def __init__(
        self,
        thresholds: list,
        clock: time.clock,
        show: bool = True,
        max_untracked_frames: int = 0,
        dynamic_threshold: bool = False,
        threshold_update_rate: int = 0,
    ) -> None:
        """
        @description: Constructor of the Tracker class
        @param       {*} self:
        @param       {list} thresholds: The list of thresholds for goal or balloon
        @param       {time} clock: The clock to track the time
        @param       {bool} show: Whether to show the image (default: True)
        @param       {int} max_untracked_frames: The maximum number of untracked frames until the tracker resets (default: 0)
        @param       {bool} dynamic_threshold: Whether to use dynamic threshold (default: False)
        @param       {int} threshold_update_rate: The rate of threshold update (default: 0)
        @return      {*} None
        """
        self.original_thresholds = [threshold for threshold in thresholds]  # Deep copy the thresholds
        self.current_thresholds = [threshold for threshold in thresholds]  # Deep copy the thresholds
        self.clock = clock  # The clock to track the time
        self.show = show  # Whether to show the image
        self.max_untracked_frames = max_untracked_frames  # The maximum number of untracked frames
        self.dynamic_threshold = dynamic_threshold  # Whether to use dynamic threshold
        self.threshold_update_rate = threshold_update_rate  # The rate of threshold update
        self.r_LED = LED(1)  # The red LED
        self.g_LED = LED(2)  # The green LED
        self.b_LED = LED(3)  # The blue LED

    def track(self):
        # TODO: Implement this function in the child class
        pass

    def find_reference(self):
        # TODO: Implement this function in the child class
        pass

    def draw_initial_blob(self, img: image, blob: image.blob, sleep_us: int = 50000) -> None:
        """
        @description:
        @param       {image} img: The image to be drawn on
        @param       {image.blob} blob: The blob to be drawn
        @param       {int} sleep_us: The time to sleep after drawing the blob (default: 500000)
        @return      {*} None
        """
        if not blob or sleep_us < 41000:
            # No need to show anything if we do not want to show
            # sleep_us is beyond human's 24fps classy eyes' capability
            return
        else:
            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
            img.draw_rectangle(blob.rect())
            img.draw_cross(blob.cx(), blob.cy())
            img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)
            # Sleep for 500ms for initial blob debut
            time.sleep_us(sleep_us)

    def _find_max(self, nice_blobs: list) -> image.blob:
        """
        @description: Find the blob with the largest area
        @param       {list} nice_blobs: The list of blobs to be compared
        @return      {image.blob} The blob with the largest area
        """
        max_blob = None
        max_area = 0
        for blob in nice_blobs:
            if blob.area() > max_area:
                max_blob = blob
                max_area = blob.pixels()
        return max_blob

    def _comp_new_threshold(self, statistics: image.statistics, mul_stdev: float = 3) -> tuple:
        """
        @description: Compute the new threshold based on the color statistics
        @param       {image} statistics: The color statistics of the blob
        @param       {float} mul_stdev: The multiplier of the standard deviation
        @return      {tuple} The new threshold
        WARNING: Could be deprecated
        """
        L_mean = statistics.l_mean()
        L_stdev = statistics.l_stdev()
        A_mean = statistics.a_mean()
        A_stdev = statistics.a_stdev()
        B_mean = statistics.b_mean()
        B_stdev = statistics.b_stdev()

        new_threshold = (
            L_mean - mul_stdev * L_stdev,
            L_mean + mul_stdev * L_stdev,
            A_mean - mul_stdev * A_stdev,
            A_mean + mul_stdev * A_stdev,
            B_mean - mul_stdev * B_stdev,
            B_mean + mul_stdev * B_stdev,
        )
        return new_threshold

    def _comp_weighted_avg(self, new_vec: tuple, orig_vec: tuple, w1: float = 0.1, w2: float = 0.9) -> tuple:
        """
        @description: Compute the weighted average of two vectors
        @param       {tuple} new_vec: The new vector
        @param       {tuple} orig_vec: The original vector to be averaged with
        @param       {float} w1: The weight of the new vector (default: 0.1)
        @param       {float} w2: The weight of the original vector (default: 0.9)
        @return      {tuple} The weighted average of the two vectors
        WARNING: Could be deprecated
        """
        weighted_avg = [int(w1 * new_vec[i] + w2 * orig_vec[i]) for i in range(len(new_vec))]
        return tuple(weighted_avg)

    def update_thresholds(
        self,
        statistics: image.statistics = None,
        recall: bool = False,
        reset: bool = False,
    ) -> None:
        """
        @description: Update the thresholds
        @param       {*} self:
        @param       {image.statistics} statistics: The color statistics of the blob
        @param       {bool} recall: If we want to recall the original threshold (default: False)
        @param       {bool} reset: If we want to reset the threshold (default: False)
        @return      {*} None
        """
        if not self.dynamic_threshold:
            return
        if recall:
            new_threshold = [threshold for threshold in self.original_thresholds]  # Deep copy the original thresholds
        else:
            new_threshold = self._comp_new_threshold(statistics) if statistics else self.original_thresholds
        # Calculate the weighted average of the new threshold and the original threshold
        for i in range(len(self.current_thresholds)):
            self.current_thresholds[i] = self._comp_weighted_avg(
                new_threshold,
                self.current_thresholds[i],
                self.threshold_update_rate if not reset else 0,
                1 - self.threshold_update_rate if not reset else 1,
            )

    def update_leds(self, tracking: bool = False, detecting: bool = False, lost: bool = True) -> None:
        """
        @description: Update the LEDs state
        @param       {*} self:
        @param       {bool} tracking: If we are tracking the blob in the roi
        @param       {bool} detecting: If we are actually detecting the blob
        @param       {bool} lost: If we lost the blob
        @return      {*} None
        """
        if tracking and detecting and not lost:
            self.g_LED.off()
            self.r_LED.off()
            self.b_LED.on()
        elif tracking and not detecting and not lost:
            self.g_LED.off()
            self.b_LED.on()
            self.r_LED.on()
        elif lost:
            self.b_LED.off()
            self.r_LED.off()
            self.g_LED.on()
        else:
            print("Error: Invalid LED state")
            pass

# tracking bounding box for the goal
class GoalTracker(Tracker):
    def __init__(
        self,
        thresholds: list,
        clock: time.clock,
        show: bool = True,
        max_untracked_frames: int = 8,
        dynamic_threshold: bool = False,
        threshold_update_rate: float = 0,
        LEDpin: str = "PG12",
        sensor_sleep_time: int = 50000,
    ) -> None:
        """
        @description:
        @param       {*} self:
        @param       {list} thresholds: The list of thresholds for the goal
        @param       {time} clock: The clock to track the time
        @param       {bool} show: Whether to show the image (default: True)
        @param       {int} max_untracked_frames: The maximum number of untracked frames until the tracker resets (default: 5)
        @param       {bool} dynamic_threshold: Whether to use dynamic threshold (default: False)
        @param       {float} threshold_update_rate: The rate of threshold update (default: 0)
        @param       {str} LEDpin: The pin of the IR LED (default: "PG12")
        @param       {int} sensor_sleep_time: The time to sleep after the sensor captures a new image (default: 50000)
        @return      {*}
        """
        super().__init__(
            thresholds,
            clock,
            show,
            max_untracked_frames,
            dynamic_threshold,
            threshold_update_rate,
        )
        self.shape_detector = ShapeDetector(gridsize=9)
        self.LED_STATE = False
        self.time_last_snapshot = time.time_ns()  # wait for the sensor to capture a new image
        self.extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
        self.extra_fb2 = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
        self.extra_fb3 = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
        self.IR_LED = Pin(LEDpin, Pin.OUT)
        self.IR_LED.value(0)
        self.roi = MemROI(ffp=0.30, ffs=0.08, gfp=1, gfs=0.9)  # The ROI of the blob
        self.tracked_blob = None
        blob, _ = self.find_reference()
        self.tracked_blob = CurBLOB(blob)
        self.flag = 0x80
        self.sensor_sleep_time = sensor_sleep_time

    def __del__(self):
        sensor.dealloc_extra_fb()
        sensor.dealloc_extra_fb()
        sensor.dealloc_extra_fb()
        self.shape_detector.__del__()


    def track(self, edge_removal: bool = True) -> tuple:
        """
        @description: Track the blob with dynamic threshold and ROI
        @param       {*} self:
        @param       {bool} edge_removal: Whether to remove the edge noises (default: True)
        @return      {tuple} The feature vector of the tracked blob and whether the blob is tracked
        """
        # the 8-bit flag variable
        # From MSB to LSB
        # [7]: 1 for goal
        # [6]: reserved for balloons
        # [5:2]: reserved
        # [1:0]: toggling between 1 and 2 for new detections, 0 for no detection
        self.update_leds(tracking=True, detecting=True, lost=False)  # Set the LEDs to indicate tracking

        # Initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            self.flag = 0x80
            self.update_leds(tracking=False, detecting=False, lost=True)  # Set the LEDs to indicate tracking
            reference_blob, statistics = self.find_reference(time_show_us=0)  # Find the blob with the largest area
            if reference_blob:
                self.tracked_blob.reinit(reference_blob)  # Initialize the tracked blob with the reference blob
                # median_lumen = statistics.median()
                # if median_lumen <= self.current_thresholds[0][1]:
                #     # solid!
                #     flag |= 0x04
                self.update_thresholds(statistics)  # Update the dynamic threshold
                self.roi.update(self.tracked_blob.feature_vector[0:4])  # Update the ROI
                self.update_leds(tracking=True, detecting=True, lost=False)
                # color_id = self.tracked_blob.blob_history[-1].code()
                # if color_id & 0b1:
                #     # green
                #    flag |= 0b10
                # elif color_id & 0b10:
                #     # orange
                #     flag &= 0xfd
                self.flag |= 0x01
                return self.tracked_blob.feature_vector, self.flag
            else:
                return None, self.flag

        # Track the blob
        img, list_of_blobs = self.detect(isColored=True, edge_removal=edge_removal)
        blob_rect = self.tracked_blob.update(list_of_blobs)

        if self.tracked_blob.untracked_frames >= self.max_untracked_frames:
            # If the blob fails to track for self.max_untracked_frames frames,
            # reset the tracking and find a new reference blob
            self.update_leds(tracking=False, detecting=False, lost=True)
            self.tracked_blob.reset()
            # self.roi.reset() (NOTE: ROI is not reset since we are assuming that the blob tends to appear in the same region when it is lost)
            print("Goal lost")
            self.update_thresholds(reset=True)
            self.flag = 0x80
            return None, self.flag
        elif self.flag == 0x80:
            self.flag = 0x81
        else:
            flag_toggling = self.flag & 0x03
            flag_toggling = 3 - flag_toggling
            self.flag = 0x80 | flag_toggling
        if blob_rect:
            # color_id = self.tracked_blob.blob_history[-1].code()
            # if color_id & 0b1:
            #     # green
            #     flag |= 0b10
            # elif color_id & 0b10:
            #     # orange
            #     flag &= 0xfd
            # If we discover the reference blob again
            self.roi.update(blob_rect)
            # We wnat to have a focus on the center of the blob
            shurnk_roi = list(blob_rect)
            shurnk_roi[0] += round(0.25*shurnk_roi[2])
            shurnk_roi[1] += round(0.25*shurnk_roi[3])
            shurnk_roi[2] //= 2
            shurnk_roi[3] //= 2
            statistics = img.get_statistics(roi=shurnk_roi)
            # median_lumen = statistics.median()
            # if median_lumen <= self.current_thresholds[0][1]:
            #     # solid!
            #     flag |= 0x04
            self.update_thresholds(statistics)
            self.update_leds(tracking=True, detecting=True, lost=False)
        else:
            # If we do not discover the reference blob
            self.update_leds(tracking=True, detecting=False, lost=False)
            self.roi.update()
            self.update_thresholds(recall=True)
        if self.show:
            x0, y0, w, h = [math.floor(self.tracked_blob.feature_vector[i]) for i in range(4)]
            img.draw_rectangle(x0, y0, w, h, color=(255, 0, 0))
            img.draw_rectangle(self.roi.get_roi(), color=(128, 128, 0))
            img.flush()
        return self.tracked_blob.feature_vector, self.flag

    def find_reference(
        self,
        time_show_us: int = 50000,
    ) -> tuple:
        """
        @description: Find the a good blob to be the reference blob
        @param       {*} self:
        @param       {int} time_show_us: The time to show the blob on the screen
        @return      {tuple} The reference blob and its color statistics
        """
        omv.disable_fb(False)

        img, nice_blobs = self.detect(isColored=True, edge_removal=False)
        img.flush()
        if not nice_blobs:
            return None, None

        best_blob = self._find_max(nice_blobs)  # Find the best blob, will never return None if nice_blobs is not empty
        self.draw_initial_blob(img, best_blob, time_show_us)  # Draw the initial blob
        omv.disable_fb(True)

        # We wnat to have a focus on the center of the blob
        shurnk_roi = list(best_blob.rect())
        shurnk_roi[0] += round(0.25*shurnk_roi[2])
        shurnk_roi[1] += round(0.25*shurnk_roi[3])
        shurnk_roi[2] //= 2
        shurnk_roi[3] //= 2

        statistics = img.get_statistics(roi=shurnk_roi)  # Get the color statistics of the blob in actual image
        return best_blob, statistics

    def detect(self, isColored=False, edge_removal=True):
        omv.disable_fb(True)  # No show on screen
        # Get an extra frame buffer and take a snapshot
        self.clock.tick()
        ##################################################################
        self.LED_STATE = True
#        if self.tracked_blob is not None:
#            if self.tracked_blob.untracked_frames > 1:
#                self.LED_STATE = not self.LED_STATE
#            elif self.tracked_blob.blob_history is None:
#                self.LED_STATE = not self.LED_STATE

#        else:
#            self.LED_STATE = not self.LED_STATE
#        print("no")
#        self.sensor_sleep(self.time_last_snapshot)
        self.IR_LED.value(not self.LED_STATE)
        sensor.skip_frames(1)
        while(not sensor.get_frame_available()):
            pass
#        time.sleep_us(20)

#        self.time_last_snapshot = time.time_ns()   # wait for the sensor to capture a new image

#        self.sensor_sleep(self.time_last_snapshot)
        self.extra_fb.replace(sensor.snapshot())
        self.time_last_snapshot = time.time_ns() # wait for the sensor to capture a new image
        ###################################################################

        self.IR_LED.value(self.LED_STATE)
#        self.sensor_sleep(self.time_last_snapshot)
#        sensor.skip_frames(1)

        while(not sensor.get_frame_available()):
            pass
#        time.sleep_us(20)
#            time.sleep_us(1)
#        time.sleep_us(int(self.sensor_sleep_time/2))


        self.IR_LED.value(not self.LED_STATE)
        self.extra_fb2.replace(sensor.snapshot())
        self.time_last_snapshot = time.time_ns()  # wait for the sensor to capture a new image
        ######################################################################
#        self.sensor_sleep(self.time_last_snapshot)

#        sensor.skip_frames(1)
        while(not sensor.get_frame_available()):
            pass
#        time.sleep_us(20)
#            time.sleep_us(1)

        img = sensor.snapshot()
        self.time_last_snapshot = time.time_ns()  # wait for the sensor to capture a new image

        #IR_LED.value(not LED_STATE)

        #original
        self.IR_LED.value(False)
        img.sub(self.extra_fb2, reverse=self.LED_STATE)
        self.extra_fb3.replace(img)
        img.replace(self.extra_fb)
        img.sub(self.extra_fb2, reverse=self.LED_STATE)
        img.difference(self.extra_fb3)
        self.extra_fb2.replace(img)
        img.replace(self.extra_fb3)
        img.sub(self.extra_fb2, reverse=self.LED_STATE)



#        self.IR_LED.value(False)
#        img.sub(self.extra_fb2, reverse=self.LED_STATE)
#        self.extra_fb3.replace(img)
#        img.replace(self.extra_fb)
#        img.sub(self.extra_fb2, reverse=self.LED_STATE)
#        img.difference(self.extra_fb3)
#        self.extra_fb2.replace(img)
#        img.replace(self.extra_fb3)
#        img.sub(self.extra_fb2, reverse=self.LED_STATE)
        # Remove the edge noises


        img.negate()
        list_of_blob = img.find_blobs(
            self.current_thresholds,
            area_threshold=3,
            pixels_threshold=3,
            margin=5,
            x_stride=1,
            y_stride=1,
            merge=True,

        )
#        largest_blob = max(list_of_blob, key=lambda b: b.area(), default=None)

        #shape detection/determination
#        if largest_blob:
#            roi_img, roi = self.shape_detector.extract_valid_roi(img, largest_blob, self.current_thresholds)
#            if roi_img:
#                detected_shape = self.shape_detector.detect_shape(roi_img)
#                print("Detected Shape:", detected_shape)


##                mean_pooled_img = self.shape_detector.downsample_and_average(roi_img)
##                gridsize = 9
##    #             Visually represent the data (example code)
##                scale_x = roi[2] / gridsize
##                scale_y = roi[3] / gridsize
##                for i in range(gridsize):
##                    for j in range(gridsize):
##                        gray_value = mean_pooled_img.get_pixel(j, i) *255
##                        rect_x = roi[0] + j * int(scale_x)
##                        rect_y = roi[1] + i * int(scale_y)
##                        rect_width = max(int(scale_x), 1)
##                        rect_height = max(int(scale_y), 1)
##                        img.draw_rectangle(rect_x, rect_y, rect_width, rect_height, color=(gray_value, gray_value, gray_value), fill=True)
#                img.draw_rectangle(largest_blob.rect(), color=(127, 0, 127))  # Highlight the blob
#                img.draw_string(largest_blob.x(), largest_blob.y(), detected_shape, color=(255, 0, 255))
        st = "FPS: {}".format(str(round(self.clock.fps(), 2)))
        img.draw_string(0, 0, st, color=(255, 0, 0))
        # sensor.dealloc_extra_fb()
        omv.disable_fb(False)
        big_blobs=[]
        for blob in list_of_blob:
            if blob.area() > 40 and line_length(blob.minor_axis_line())> 10:
                roi_img, roi = self.shape_detector.extract_valid_roi(img, blob, self.current_thresholds)
                if roi_img:
                    detected_shape = self.shape_detector.detect_shape(roi_img)
                    if detected_shape != "triangle" and detected_shape != "not":
                        big_blobs.append(blob)
                    img.draw_string(blob.x(), blob.y(), detected_shape[0], color=(255, 0, 255))
                    img.draw_rectangle(blob.rect(), color=(255, 0, 255))
                    del(roi_img)
#                else: # if shape cannot be determined add blob to anyway
#                    big_blobs.append(blob)
#                    img.draw_rectangle(blob.rect(), color=(255, 0, 0))  # Red rectangle around the blob for visibility
#            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
#            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
#            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
#            img.draw_rectangle(blob.rect())
#            img.draw_cross(blob.cx(), blob.cy())
#            img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)


        return img, big_blobs

    def sensor_sleep(self, last_time_stamp) -> None:
        """
        @description: Wait for the sensor for some time from the last snapshot to avoid a partial new image
        @param       {*} last_time_stamp: The time stamp of the last snapshot
        @return      {*} None
        """
        elapsed = self.sensor_sleep_time - (int((time.time_ns() - last_time_stamp) / 1000))
        while elapsed < 0:
            elapsed += self.sensor_sleep_time
#        if elapsed >= 0:
        time.sleep_us(elapsed)
#        else:
#            time.sleep_us(self.sensor_sleep_time+ elapsed%self.sensor_sleep_time)
        return None

# helper function for calculating the length of a line segment
def line_length(coords):
    """Calculate the length of a line segment given its coordinates.

    Args:
    coords (tuple): A tuple of four elements (x1, y1, x2, y2) representing
                    the coordinates of the two endpoints of the line segment.

    Returns:
    float: The length of the line segment.
    """
    x1, y1, x2, y2 = coords
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


""" Sensor initialization based on the object to track: 0 for balloon and 1 for goal """
def init_sensor_target(tracking_type:int, framesize=sensor.HQVGA, windowsize=None) -> None:
    """ Initialize sensors by updating the registers
        for the two different purposes
        @param       {int} tracking_type: 0 for balloons and 1 for goals
        @return      {*} None
    """
    if tracking_type == 1:
        # goal detection sensor setup
        sensor.reset()
        sensor.set_auto_exposure(False)
        sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
        sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
        sensor.__write_reg(0x80, 0b01111110) # [7] reserved, [6] gamma enable, [5] CC enable,
        sensor.__write_reg(0x03, 0b00000011) # high bits of exposure control
        sensor.__write_reg(0x04, 0b11101000) # low bits of exposure control
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(framesize)
        if ADVANCED_SENSOR_SETUP:
            sensor.__write_reg(0x80, 0b01101110)    # [7] reserved, [6] gamma enable, [5] CC enable,
                                                    # [4] Edge enhancement enable
                                                    # ------------------------------------------------
                                                    # WARNING: necessary or unusable image will occur:
                                                    # [3] Interpolation enable,
                                                    # ------------------------------------------------
                                                    # [2] DN enable, [1] DD enable,
                                                    # ------------------------------------------------
                                                    # WARNING: extremely recommended to disable:
                                                    # [0] Lens-shading correction enable - gives you uneven
                                                    #                                      shade in the dark
                                                    #                                      badly!!!!!
                                                    # ------------------------------------------------
            sensor.__write_reg(0x81, 0b01010100)    # [7] BLK dither mode, [6] low light Y stretch enable

            # ABS - anti-blur
            sensor.__write_reg(0xfe, 1)             # change to registers at page 1
            sensor.__write_reg(0x9a, 0b11110111)    # [7:4] add dynamic range, [2:0] abs adjust every frame
            sensor.__write_reg(0x9d, 0xff)          # [7:0] Y stretch limit

        # color setup - saturation
        sensor.__write_reg(0xfe, 2)             # change to registers at page 2
        sensor.__write_reg(0xd0, SATURATION)    # change global saturation - default 64
        sensor.__write_reg(0xd1, 48)            # Cb saturation - default 48
        sensor.__write_reg(0xd2, 48)            # Cr saturation - default 48
        sensor.__write_reg(0xd3, CONTRAST)            # contrast - default 48
        sensor.__write_reg(0xd5, 0)             # luma offset - default 0 (sign + 7 format)
        sensor.skip_frames(time = 1000)

    elif tracking_type == 0:
        # balloon detection sensor setup
        sensor.reset()
        sensor.set_auto_whitebal(True)
        sensor.set_auto_exposure(True)
        sensor.set_pixformat(sensor.RGB565)
        sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True) # wide FOV
        sensor.set_framesize(sensor.HQVGA)

        sensor.set_auto_whitebal(False)
        sensor.set_auto_exposure(False)

        """ sensor setup
        """

        sensor.__write_reg(0xfe, 0) # change to registers at page 0
        sensor.__write_reg(0x80, 0b01111110)    # [7] reserved, [6] gamma enable, [5] CC enable,
                                                # [4] Edge enhancement enable
                                                # [3] Interpolation enable, [2] DN enable, [1] DD enable,
                                                # [0] Lens-shading correction enable - gives you uneven
                                                #                                      shade in the dark
                                                #                                      badly!!!!!
        sensor.__write_reg(0x81, 0b01010100)    # [7] BLK dither mode, [6] low light Y stretch enable
                                                # [5] skin detection enable, [4] reserved, [3] new skin mode
                                                # [2] autogray enable, [1] reserved, [0] BFF test image mode
        sensor.__write_reg(0x82, 0b00000100)    # [2] ABS enable, [1] AWB enable
        #sensor.__write_reg(0x87, 0b00000001)    # [0] auto_edge_effect
        sensor.__write_reg(0x9a, 0b00001111)    # [3] smooth Y, [2] smooth Chroma,
                                                # [1] neighbor average mode, [0] subsample extend opclk
        sensor.skip_frames(time = 1000)
        print("block enabling done")

        # Edge enhancements
        sensor.__write_reg(0xfe, 2)             # change to registers at page 2
        sensor.__write_reg(0x90, 0b11101101)    # [7]edge1_mode, [6]HP3_mode, [5]edge2_mode, [4]Reserved,
                                                # [3]LP_intp_en, [2]LP_edge_en, [1]NA, [0] half_scale_mode_en
        sensor.__write_reg(0x91, 0b11000000)    # [7]HP_mode1, [6]HP_mode2,
                                                # [5]only 2 direction - only two direction H and V, [4]NA
                                                # [3]only_defect_map, [2]map_dir, [1:0]reserved
        sensor.__write_reg(0x96, 0b00001100)    # [3:2] edge leve
        sensor.__write_reg(0x97, 0x88)          # [7:4] edge1 effect, [3:0] edge2 effect
        sensor.__write_reg(0x9b, 0b00100010)    # [7:4] edge1 threshold, [3:0] edge2 threshold
        sensor.skip_frames(time = 1000)
        print("edge enhancement done")

        # color correction -- this is very tricky: the color shifts on the color wheel it seems
        sensor.__write_reg(0xfe, 2) # change to registers at page 2
        # WARNING: uncomment the two lines to invert the color
        #sensor.__write_reg(0xc1, 0x80)          # CC_CT1_11, feels like elements in a matrix
        #sensor.__write_reg(0xc5, 0x80)          # CC_CT1_22 , feels like elements in a matrix
        print("color correction setup done")

        # ABS - anti-blur
        sensor.__write_reg(0xfe, 1)             # change to registers at page 1
        sensor.__write_reg(0x9a, 0b11110111)    # [7:4] add dynamic range, [2:0] abs adjust every frame
        sensor.__write_reg(0x9d, 0xff)          # [7:0] Y stretch limit
        sensor.skip_frames(time = 1000)
        print("anti-blur setup done")


        # color settings -- AWB
        """ Ranting about the trickiness of the setup:
            Even the auto white balance is disabled, the AWB gains will persist to
            take effect. Although the correcponding registers are read-only in the
            document, they are actually manually writeable, and such writings are
            effective. On top of these messes, another set of registers that are
            R/W have the exact same effect on the RGB gains, but they are not
            controlled by the AWB.
        """
        sensor.set_auto_exposure(False)
        sensor.set_auto_whitebal(False) # no, the gain_rgb_db does not work

        # reset RGB auto gains
        sensor.__write_reg(0xb3, 64)    # reset R auto gain
        sensor.__write_reg(0xb4, 64)    # reset G auto gain
        sensor.__write_reg(0xb5, 64)    # reset B auto gain

        sensor.__write_reg(0xfe, 0)     # change to registers at page 0
                                        # manually set RGB gains to fix color/white balance
        sensor.__write_reg(0xad, R_GAIN)    # R gain ratio
        sensor.__write_reg(0xae, G_GAIN)    # G gain ratio
        sensor.__write_reg(0xaf, B_GAIN)    # B gain ratio
        sensor.set_auto_exposure(True)
        sensor.skip_frames(time = 1000)
        print("AWB Gain setup done.")


        # color setup - saturation
        sensor.__write_reg(0xfe, 2)     # change to registers at page 2
        sensor.__write_reg(0xd0, 128)    # change global saturation,
        sensor.__write_reg(0xd1, 48)    # Cb saturation
        sensor.__write_reg(0xd2, 48)    # Cr saturation
        sensor.__write_reg(0xd3, 40)    # contrast
        sensor.__write_reg(0xd5, 0)     # luma offset
    else:
        raise ValueError("Not a valid sensor-detection mode!")

    if windowsize is not None:
        sensor.set_windowing(windowsize)

""" IBus communication functions """
# checksum that we can but we are not using on the ESP side for verifying data integrity
def checksum(arr, initial= 0):
    """ The last pair of byte is the checksum on iBus
    """
    sum = initial
    for a in arr:
        sum += a
    checksum = 0xFFFF - sum
    chA = checksum >> 8
    chB = checksum & 0xFF
    return chA, chB

# send an ibus message array to uart, each element is 2-byte
# for some of them we are only using 1 byte anyways
def IBus_message(message_arr_to_send):
    msg = bytearray(32)
    msg[0] = 0x20
    msg[1] = 0x40
    for i in range(len(message_arr_to_send)):
        msg_byte_tuple = bytearray(message_arr_to_send[i].to_bytes(2, 'little'))
        msg[int(2*i + 2)] = msg_byte_tuple[0]
        msg[int(2*i + 3)] = msg_byte_tuple[1]

    # Perform the checksume
    chA, chB = checksum(msg[:-2], 0)
    msg[-1] = chA
    msg[-2] = chB
    return msg


def mode_initialization(input_mode, mode, grid=None, detectors=None):
    """ Switching between blinking goal tracker and balloon tracker
    """
    if mode == input_mode:
        print("already in the mode")
        return None
    else:
        if input_mode == 0:
            # balloon tracking mode
            init_sensor_target(tracking_type=0)
            tracker = BalloonTracker(grid, detectors)
            print("balloon mode!")
        elif input_mode == 1:
            thresholds = TARGET_COLOR
            init_sensor_target(tracking_type=1)
            # Find reference
            tracker = GoalTracker(thresholds, clock, max_untracked_frames = 10, sensor_sleep_time=WAIT_TIME_US)
            print("Goal mode!")
        else:
            raise ValueError("Invalid mode selection")

        return input_mode, tracker


if __name__ == "__main__":
    """ Necessary for both modes """
    clock = time.clock()
    mode = 0 # 0 for balloon detection and 1 for goal

    # Initialize inter-board communication
    # time of flight sensor initialization
    # tof = VL53L1X(I2C(2)) # seems to interfere with the uart

    """ Grid detection setup for balloons """
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True) # wide FOV
    sensor.set_framesize(sensor.HQVGA)
    sensor.skip_frames(time = 1000)
    img = sensor.snapshot()
    grid = Grid(N_ROWS, N_COLS, img.width(), img.height())

    # Color distance
    purpleDet = ColorDetector("Purple", line_ref = COLOR_LINE_REF_PURPLE,
                              max_dist=MAX_DIST_PURPLE, std_range=STD_RANGE_PURPLE,
                              rgb=(255,0,255), mahalanobis=True,
                              mu=COLOR_PURPLE_MEAN,
                              sigma_inv=COLOR_PURPLE_INV_COV)

    greenDet = ColorDetector("Green", line_ref=COLOR_LINE_REF_GREEN,
                            max_dist=MAX_DIST_GREEN, std_range=STD_RANGE_GREEN,
                            rgb=(0,255,0), mahalanobis=False, mu=None, sigma_inv=None)
    blueDet = ColorDetector("Blue", line_ref=COLOR_LINE_REF_BLUE,
                            max_dist=MAX_DIST_BLUE, std_range=STD_RANGE_BLUE,
                            rgb=(0, 0, 255),mahalanobis=False, mu=None, sigma_inv=None)
    detectors = [purpleDet, greenDet, blueDet]


    # Initializing the tracker
    mode, tracker = mode_initialization(mode, -1, grid, detectors)
    del img
    img = None

    # Initialize UART
    uart = UART("LP1", baudrate= 115200, timeout_char=10) # (TX, RX) = (P1, P0) = (PB14, PB15) = "LP1"

    """ Main loop """
    while True:
        clock.tick()
        if mode == 0:
            if BACKLIGHT_TOO_MUCH:
                # semi automatic auto exposure for higher brightness
                if exp_counter == 1:
                    sensor.set_auto_exposure(True)
                elif exp_counter == CAP_EXP_COUNTER//AES_EVERY:
                    sensor.set_auto_exposure(False)
                    sensor.__write_reg(0xfe, 0b00000000)    # change to registers at page 0
                    high_exp = sensor.__read_reg(0x03)  # high bits of exposure control
                    low_exp = sensor.__read_reg(0x04)   # low bits of exposure control
                    exposure = int(((high_exp << 8) + low_exp) * BRIGHTNESS_MULT)
                    sensor.__write_reg(0x03, exposure >> 8) # force BRIGHTNESS_MULT x exposure time
                    sensor.__write_reg(0x04, exposure & 0xff) # force BRIGHTNESS_MULT x exposure time
                elif exp_counter == CAP_EXP_COUNTER:
                    exp_counter = 0
                exp_counter += 1
        if mode == 1:
            feature_vector, flag = tracker.track()
        else:
            img = sensor.snapshot()
            feature_vector, flag = tracker.track(img)
        try: dis = 9999
        except: dis = 9999

        if flag & 0b10000000:
            if flag & 0x03:
                roi = tracker.roi.get_roi()
                feature_vec = tracker.tracked_blob.feature_vector
                x_roi = roi[0] + roi[2]//2
                y_roi = roi[1] + roi[3]//2
                w_roi = roi[2]
                h_roi = roi[3]

                x_value = int(feature_vec[0] + feature_vec[2]/2)
                y_value = int(feature_vec[1] + feature_vec[3]/2)
                w_value = int(feature_vec[2])
                h_value = int(feature_vec[3])
                msg = IBus_message([flag, x_roi, y_roi, w_roi, h_roi,
                                    x_value, y_value, w_value, h_value, dis])
            else:
                msg = IBus_message([flag, 0, 0, 0, 0, 0, 0, 0, 0, dis])
        elif flag & 0b01000000:
            x_roi, y_roi, w_roi, h_roi, x_value, y_value, w_value, h_value, just_zero = feature_vector
            msg = IBus_message([flag, x_roi, y_roi, w_roi, h_roi,
                                x_value, y_value, w_value, h_value, dis])
        else:
            print("0 flag!")
            assert(flag == 0)

        print("fps: ", clock.fps())


        uart.write(msg)
        if uart.any():
            uart_input = uart.read();
            print(uart_input)
            if uart_input[-1] == 0x40 and mode == 1:
                tracker.__del__()
                res = mode_initialization(0, mode, grid, detectors)
                if res:
                    mode, tracker = res
            elif uart_input[-1] == 0x80 and mode == 0:
                res = mode_initialization(1, mode)
                if res:
                    mode, tracker = res
            else:
                print("possible mode transition error")
