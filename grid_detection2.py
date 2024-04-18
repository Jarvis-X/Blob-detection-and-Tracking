# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Color Binary Filter Example
#
# This script shows off the binary image filter. You may pass binary any
# number of thresholds to segment the image by.

import array
import sensor, image
import time
import math
import random
#from lib.Ibus import IBus
from pyb import UART
from pyb import LED

# Grid setup
N_ROWS = 10
N_COLS = 15

# Probablistic filter for detection
FILTER = True

# print the stats at the upper left corner
PRINT_CORNER = True

# whether combine detction of green and purple as target balloons
COMBINE_GREEN_AND_PURPLE = True

# whether remove blue color from purple color
SEPARATE_BLUE_AND_PURPLE = True

# manual white balance - to be used with *get_gains.py* in the repository
# - see RGB gain readings in the console
R_GAIN, G_GAIN, B_GAIN = 64, 64, 91

# reference line segments for different colors of balloons
COLOR_LINE_REF_PURPLE = [[26, -40], [11, -20]]
COLOR_LINE_REF_GREEN = [[-15, 13], [-19, 17]]
COLOR_LINE_REF_BLUE = [[20, -46], [1, -4]]


# Color distribution
COLOR_PURPLE_MEAN = [14.35135135135135, -30.054054054054053]
COLOR_PURPLE_INV_COV =  [[0.1489165613339765, 0.07975945458490068], [0.07975945458490068, 0.06203345185897681]]
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


# sensor setup
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


    def update(self, measurements, l_max=10, l_min=-8):
        for i, z in enumerate(measurements):

            if z < 0.1:
                li = self.l_ndet
            else:
                # The probability of being detected goes from 0.5-1. Not being detected is smaller than 0.5
                p_x = min(0.99, max(0.5 + z / 2, 0.001))
                li = math.log(p_x / (1. - p_x))
                #li = self.l_det

            # Detected or not detected
            #li = self.l_det if z else self.l_ndet
            # Belief for li
            self.L[i]+= li -self.init_belif
            # Cap
            self.L[i] = min(l_max, max(l_min, self.L[i]))


        return self.L

    def probabilities(self):
        for i, l in enumerate(self.L):
                self.P[i] = 1. / (1. + math.exp(-l))
        return self.P








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


class ColorDetector:
    def __init__(self, color_id, line_ref, max_dist, std_range, rgb, mahalanobis, mu=None, sigma_inv=None, decay = 4.):

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
            d2 = self._distance_mahalanobis((a, b))

            # Clamp max distance
            # max_num_std = 5
            # d = d if d < max_num_std else max_num_std

            # d = 1 - d / max_num_std

            # Compute the coefficient
            coefficient = 4. #/ (2 * math.pi * math.sqrt(self.determinant))
            # Compute the PDF value
            d = coefficient * math.exp(-d2 / self.decay)
            # print(d, end=", ")
        else:
            d = self._distance_point_to_segment((a, b), self.line_ref)

            # Clamp max distance
            d = d if d < self.max_dist else self.max_dist

            # Distance is 1 if close to the line, and 0 if outside the line
            d = 1 - d / self.max_dist

        # Check if standard deviation is in range
#        for s in std:
#            if not self.std_range[0] < s < self.std_range[1]:
#                d = 0.0
#                break

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
        # mahalanobis_dist = mahalanobis_sq ** 0.5
        return mahalanobis_sq

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


print("Start")





if __name__ == "__main__":
    # uart settings
    uart = UART("LP1", 115200, timeout_char=2000) # (TX, RX) = (P1, P0) = (PB14, PB15)

    # LED indicator
    led_red = LED(1)
    led_green = LED(2)
    led_blue = LED(3)

    clock = time.clock()
    img = sensor.snapshot()
    grid = Grid(N_ROWS, N_COLS, img.width(), img.height())


    # Color distance
    purpleDet = ColorDetector("Purple", line_ref = COLOR_LINE_REF_PURPLE,
                              max_dist=MAX_DIST_PURPLE, std_range=STD_RANGE_PURPLE,
                              rgb=(255, 0, 255),
                              mahalanobis=True,
                              mu=COLOR_PURPLE_MEAN,
                              sigma_inv=COLOR_PURPLE_INV_COV
                              )
    greenDet = ColorDetector("Green", line_ref=COLOR_LINE_REF_GREEN,
                             max_dist=MAX_DIST_GREEN, std_range=STD_RANGE_GREEN,
                             rgb=(0,255,0), mahalanobis=False, mu=None, sigma_inv=None)
    blueDet = ColorDetector("Blue", line_ref=COLOR_LINE_REF_BLUE,
                            max_dist=MAX_DIST_BLUE, std_range=STD_RANGE_BLUE,
                            rgb=(0, 0, 255),mahalanobis=False, mu=None, sigma_inv=None)

    # flag setup
    flag = 0

    detectors = [purpleDet]#, greenDet, blueDet]

    while True:
        if BACKLIGHT_TOO_MUCH:
            if exp_counter == 1:
                sensor.set_auto_exposure(True)
                print("auto-adjusting exp")
            elif exp_counter == CAP_EXP_COUNTER//AES_EVERY:
                print("setting exp")
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

        clock.tick()

        img = sensor.snapshot()

        # Detect
        grid.count(img, detectors)
        detector = detectors[0]

        for detector in detectors:
            metric_grid = detector.metric


            if FILTER:
               metric_grid = detector.update_filter()


            grid.plot_metric(metric_grid, detector.rgb)


        # Discard purple if blue has higher probability
        if SEPARATE_BLUE_AND_PURPLE:
            new_purple = [p if p>b else 0 for p,b in zip(purpleDet.P, blueDet.P)]

        # Combine green and purple
        if COMBINE_GREEN_AND_PURPLE:
            metric_grid = [max(p,g) for p,g in zip(new_purple, greenDet.P)]

        total_score = max(metric_grid)
        ux, uy, val = grid.action(metric_grid)

        x1 = ux
        y1 = uy
#        img.draw_circle(x1, y1, 1, color=(255, 0, 0), thickness=2)
#        img.draw_circle(x1, y1, int(total_score*img.height()/4), color=(255, 0, 0), thickness=2)

        print("fps:\t", clock.fps(), end='\t')

        # Create message for ESP32
        if total_score > 0.05:
           if max(new_purple) > max(greenDet.P):
               led_red.on()
               led_blue.on()
               led_green.off()
           elif max(new_purple) < max(greenDet.P):
               led_red.off()
               led_blue.off()
               led_green.on()
           else:
               led_red.on()
               led_blue.on()
               led_green.on()
           if flag:
               flag = 3 - flag
           else:
               flag = 1
        else:
           led_red.off()
           led_blue.off()
           led_green.off()
           flag = 0

        x_roi, y_roi = x1, y1
        w_roi, h_roi = int(10.0*val), int(10.0*val)
        x_value, y_value = x1, y1

        print("x, y =", x_roi, y_roi, "flag: ", bin(flag|0x40)) #, "\t metric=", metric_grid[5*N_ROWS:6*N_ROWS])
        msg = IBus_message([flag | 0x40, x_roi, y_roi, w_roi, h_roi,
                            x_value, y_value, img.width()//N_COLS, img.height()//N_ROWS,
                            int(total_score*5)])


        # Receive message
        if uart.any():
            uart_input = uart.read()
            print(uart_input)

        # Send message
        uart.write(msg)
