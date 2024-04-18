"""
Author       : Hanqing Qi
Date         : 2023-11-04 19:02:37
LastEditors  : Hanqing Qi
LastEditTime : 2023-11-05 17:05:51
FilePath     : /Bicopter-Vision-Control/Blob Detection & Tracking V2/lib/tracker.py
Description  : The parent class for bolbtacker and goaltracker.
"""

from machine import Pin
import sensor, image
from pyb import LED
from lib.curblob import CurBLOB
from lib.memroi import MemROI
import time
import math
import omv



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
#        if (block_width == 0 or block_height ==0):
#            return self.binary_image

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
                if roi_width > img.width()/2:
                    x_scale = .5
                if roi_height > img.height()/2:
                    y_scale = .5
                roi_img = img.copy(x_scale = x_scale, y_scale = y_scale, roi=roi).binary(current_thresholds)
                return roi_img, roi

        return None, None  # Return None if no valid ROI found




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


class BLOBTracker(Tracker):
    def __init__(
        self,
        thresholds: list,
        clock: time.clock,
        show: bool = True,
        max_untracked_frames: int = 15,
        dynamic_threshold: bool = False,
        threshold_update_rate: float = 0,
    ) -> None:
        """
        @description: Constructor of the BLOBTracker class
        @param       {*} self:
        @param       {list} thresholds: The list of thresholds (green, purple)
        @param       {time} clock: The clock to track the time
        @param       {bool} show: Whether to show the image (default: True)
        @param       {int} max_untracked_frames: The maximum number of untracked frames until the tracker resets (default: 15)
        @param       {bool} dynamic_threshold: Whether to use dynamic threshold (default: False)
        @param       {float} threshold_update_rate: The rate of threshold update (default: 0)
        @return      {*} None
        """
        super().__init__(
            thresholds,
            clock,
            show,
            max_untracked_frames,
            dynamic_threshold,
            threshold_update_rate,
        )  # Initialize the parent class
        blob, _ = self.find_reference()
        self.roi = MemROI(ffp=0.01, ffs=0.025, gfp=0.5, gfs=0.5)  # The ROI of the blob
        self.tracked_blob = CurBLOB(blob)


    def track(self):
        """
        @description: Track the blob with dynamic threshold and ROI
        @return      {tuple} The feature vector of the tracked blob and whether the blob is tracked
        """
        # the 8-bit flag variable
        # From MSB to LSB
        # [7]: 1 if tracked, 0 if not
        # [6:3] : Reserved
        # [2] : 1 if solid, 0 if hole
        # [1] : 1 if green, 0 if orange
        # [0] : 1 if goal, 0 if balloon
        flag = 0x80
        # Initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            # There is no blob history, initialize the blob
            self.update_leds(tracking=False, detecting=False, lost=True)  # Set the LEDs to indicate tracking
            reference_blob, statistics = self.find_reference(time_show_us=0)  # Find the blob with the largest area
            if reference_blob:
                self.tracked_blob.reinit(reference_blob)  # Initialize the tracked blob with the reference blob
                self.update_thresholds(statistics)  # Update the dynamic threshold
                self.roi.update(self.tracked_blob.feature_vector[0:4])  # Update the ROI
                self.update_leds(tracking=True, detecting=True, lost=False)
                return self.tracked_blob.feature_vector, flag # Return the feature vector and True
            else:
                return None, 0x0

        # Track the blob

        img = sensor.snapshot()
        list_of_blobs = img.find_blobs(
            self.current_thresholds,
            merge=True,
            pixels_threshold=35,
            area_threshold=50,
            margin=10,
            roi=self.roi.get_roi(),
            x_stride=1,
            y_stride=1,
        )
        list_of_square_blobs = []
        for blob in list_of_blobs:
            if blob.h()/blob.w() < 1.5:
                list_of_square_blobs.append(blob)
        blob_rect = self.tracked_blob.update(list_of_square_blobs)

        if self.tracked_blob.untracked_frames >= self.max_untracked_frames:
            # If the blob fails to track for 15 frames, reset the tracking and find a new reference blob
            # self.roi.reset() (NOTE: ROI is not reset since we are assuming that the blob tends to appear in the same region when it is lost)
            self.tracked_blob.reset()
            self.update_leds(tracking=False, detecting=False, lost=True)
            print("Blob lost")
            self.update_thresholds(reset=True)  # Reset the dynamic threshold
            flag &= 0x7f
            return None, flag
        if blob_rect:
            # If we discover the reference blob again
            self.roi.update(blob_rect)  # Update the ROI
            self.update_leds(tracking=True, detecting=True, lost=False)
            statistics = img.get_statistics(roi=blob_rect)
            self.update_thresholds(statistics)  # Update the dynamic threshold
        else:
            # If we do not discover the reference blob
            self.update_leds(tracking=True, detecting=False, lost=False)
            # Set the LEDs to indicate tracking but not detecting
            self.roi.update()  # Reset the ROI
            self.update_thresholds(recall=True)  # Recall the original threshold

        if self.show:
            x0, y0, w, h = [math.floor(self.tracked_blob.feature_vector[i]) for i in range(4)]
            img.draw_rectangle(x0, y0, w, h)
            img.draw_rectangle(self.roi.get_roi(), color=(255, 255, 0))
        return self.tracked_blob.feature_vector, flag

    def find_reference(
        self,
        density_threshold: float = 0.25,
        roundness_threshold: float = 0.35,
        time_show_us: int = 50000,
    ) -> tuple:
        """
        @description: Find the a good blob to be the reference blob
        @param       {*} self:
        @param       {float} density_threshold: The density threshold of the blob
        @param       {float} roundness_threshold: The roundness threshold of the blob
        @param       {int} time_show_us: The time to show the blob on the screen
        @return      {tuple} The reference blob and its color statistics
        """

        nice_blobs = []  # A list of good blobs

        img = sensor.snapshot()
        list_of_blob = img.find_blobs(
            self.original_thresholds,
            merge=True,
            pixels_threshold=30,
            area_threshold=50,
            margin=20,
            x_stride=1,
            y_stride=1,
        )
        for blob in list_of_blob:
            # Find a set of good initial blobs by filtering out the not-so-dense and not-so-round blobs
            if blob.density() > density_threshold and blob.roundness() > roundness_threshold and blob.h()/blob.w() < 1.5:
                nice_blobs.append(blob)
        if nice_blobs:  # If we find a good blob, break the loop
            best_blob = self._find_max(nice_blobs)  # Find the best blob
            self.draw_initial_blob(img, best_blob, time_show_us)  # Draw the initial blob
            statistics = img.get_statistics(roi=best_blob.rect())  # Get the color statistics of the blob in actual image
            return best_blob, statistics
        else:
            return None, None


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
        self.roi = MemROI(ffp=0.08, ffs=0.04, gfp=1, gfs=0.8)  # The ROI of the blob
        self.tracked_blob = None
        blob, _ = self.find_reference()
        self.tracked_blob = CurBLOB(blob)

        self.sensor_sleep_time = sensor_sleep_time

    def track(self, edge_removal: bool = True) -> tuple:
        """
        @description: Track the blob with dynamic threshold and ROI
        @param       {*} self:
        @param       {bool} edge_removal: Whether to remove the edge noises (default: True)
        @return      {tuple} The feature vector of the tracked blob and whether the blob is tracked
        """
        # the 8-bit flag variable
        # From MSB to LSB
        # [7]: 1 if tracked, 0 if not
        # [6:3] : Reserved
        # [2] : 1 if solid, 0 if hole
        # [1] : 1 if green OR BW, 0 if orange
        # [0] : 1 if goal, 0 if balloon
        flag = 0x81

        self.update_leds(tracking=True, detecting=True, lost=False)  # Set the LEDs to indicate tracking
        # Initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            self.update_leds(tracking=False, detecting=False, lost=True)  # Set the LEDs to indicate tracking
            reference_blob, statistics = self.find_reference(time_show_us=0)  # Find the blob with the largest area
            if reference_blob:
                self.tracked_blob.reinit(reference_blob)  # Initialize the tracked blob with the reference blob
                median_lumen = statistics.median()
                if median_lumen <= self.current_thresholds[0][1]:
                    # solid!
                    flag |= 0x04
                self.update_thresholds(statistics)  # Update the dynamic threshold
                self.roi.update(self.tracked_blob.feature_vector[0:4])  # Update the ROI
                self.update_leds(tracking=True, detecting=True, lost=False)
                color_id = self.tracked_blob.blob_history[-1].code()
                if color_id & 0b1:
                    # green
                    flag |= 0b10
                elif color_id & 0b10:
                    # orange
                    flag &= 0xfd
                return self.tracked_blob.feature_vector, flag
            else:
                return None, 0x01


        # Track the blob
        img, list_of_blobs = self.detect(isColored=True, edge_removal=edge_removal)
        blob_rect = self.tracked_blob.update(list_of_blobs)

        if self.tracked_blob.untracked_frames >= self.max_untracked_frames:
            # If the blob fails to track for 15 frames, reset the tracking and find a new reference blob
            self.update_leds(tracking=False, detecting=False, lost=True)
            self.tracked_blob.reset()
            # self.roi.reset() (NOTE: ROI is not reset since we are assuming that the blob tends to appear in the same region when it is lost)
            print("Goal lost")
            self.update_thresholds(reset=True)
            flag &= 0x7f
            return None, flag
        if blob_rect:
            color_id = self.tracked_blob.blob_history[-1].code()
            if color_id & 0b1:
                # green
                flag |= 0b10
            elif color_id & 0b10:
                # orange
                flag &= 0xfd
            # If we discover the reference blob again
            self.roi.update(blob_rect)
            # We wnat to have a focus on the center of the blob
            shurnk_roi = list(blob_rect)
            shurnk_roi[0] += round(0.25*shurnk_roi[2])
            shurnk_roi[1] += round(0.25*shurnk_roi[3])
            shurnk_roi[2] //= 2
            shurnk_roi[3] //= 2
            statistics = img.get_statistics(roi=shurnk_roi)
            median_lumen = statistics.median()

            if median_lumen <= self.current_thresholds[0][1]:
                # solid!
                flag |= 0x04

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
        return self.tracked_blob.feature_vector, flag

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
#        self.LED_STATE = True
        if self.tracked_blob is not None:
            if self.tracked_blob.untracked_frames > 1:
                self.LED_STATE = not self.LED_STATE
            elif self.tracked_blob.blob_history is None:
                self.LED_STATE = not self.LED_STATE

        else:
            self.LED_STATE = not self.LED_STATE
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
            if blob.area() > 20 and line_length(blob.minor_axis_line())> 5:
                roi_img, roi = self.shape_detector.extract_valid_roi(img, blob, self.current_thresholds)
                if roi_img:
                    detected_shape = self.shape_detector.detect_shape(roi_img)
                    if detected_shape != "triangle" and detected_shape != "not":
                        big_blobs.append(blob)
                    img.draw_string(blob.x(), blob.y(), detected_shape[0], color=(255, 0, 255))
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
