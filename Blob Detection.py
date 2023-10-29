"""
Author       : Hanqing Qi & Karen Li & Jiawei Xu
Date         : 2023-10-20 17:16:42
LastEditors  : Jiawei Xu
LastEditTime : 2023-10-27 0:38:54
FilePath     :
Description  : Send the blob detection data (cx, cy, w, h) to the esp32
"""

import sensor, image, time
from pyb import LED
from pyb import UART
from machine import I2C
from machine import Pin
from vl53l1x import VL53L1X
import mjpeg, pyb
import random
import math

GREEN = [ (26, 38, -18, 0, -24, 1), (35, 58, -30, 2, -19, -4) ]
PURPLE = [ (20, 24, 4, 15, -22, -7) ]
THRESHOLD_UPDATE_RATE = 0.0

def init_sensor_target(pixformat=sensor.RGB565, framesize=sensor.HQVGA, windowsize=None) -> None:
    sensor.reset()                        # Initialize the camera sensor.
    sensor.set_pixformat(pixformat)       # Set pixel format to RGB565 (or GRAYSCALE)
    sensor.set_framesize(framesize)
    if windowsize is not None:            # Set windowing to reduce the resolution of the image
        sensor.set_windowing(windowsize)
    sensor.skip_frames(time=1000)         # Let new settings take affect.
    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)
    sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
    sensor.__write_reg(0x80, 0b10111100) # enable gamma, CC, edge enhancer, interpolation, de-noise
    sensor.__write_reg(0x81, 0b01101100) # enable BLK dither mode, low light Y stretch, autogray enable
    sensor.__write_reg(0x82, 0b00000100) # enable anti blur, disable AWB
    sensor.__write_reg(0x03, 0b00000011) # high bits of exposure control
    sensor.__write_reg(0x04, 0b11110000) # low bits of exposure control
    sensor.__write_reg(0xb0, 0b11110000) # global gain
#    sensor.__write_reg(0xad, 0b01001100) # R ratio
#    sensor.__write_reg(0xae, 0b01010100) # G ratio
#    sensor.__write_reg(0xaf, 0b01101000) # B ratio
    # RGB gains
    sensor.__write_reg(0xa3, 0b01111000) # G gain odd
    sensor.__write_reg(0xa4, 0b01111000) # G gain even
    sensor.__write_reg(0xa5, 0b10000010) # R gain odd
    sensor.__write_reg(0xa6, 0b10000010) # R gain even
    sensor.__write_reg(0xa7, 0b10001000) # B gain odd
    sensor.__write_reg(0xa8, 0b10001000) # B gain even
    sensor.__write_reg(0xa9, 0b10000000) # G gain odd 2
    sensor.__write_reg(0xaa, 0b10000000) # G gain even 2
    sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
    # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
                                           # strangely constrained by auto saturation
    sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
    sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
    sensor.__write_reg(0xd3, 0b01001000) # luma contrast
    # sensor.__write_reg(0xd5, 0b00000000) # luma offset
    sensor.skip_frames(time=2000) # Let the camera adjust.


def draw_initial_blob(img, blob, sleep_us=500000) -> None:
    """ Draw initial blob and pause for sleep_us for visualization
    """
    if not blob or sleep_us < 41000:
        # No need to show anything if we do not want to show
        # it beyond human's 24fps classy eyes' capability
        return None
    else:
        img.draw_edges(blob.min_corners(), color=(255,0,0))
        img.draw_line(blob.major_axis_line(), color=(0,255,0))
        img.draw_line(blob.minor_axis_line(), color=(0,0,255))
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)
        print(blob.cx(), blob.cy(), blob.w(), blob.h(), blob.pixels())
        # sleep for 500ms for initial blob debut
        time.sleep_us(sleep_us)


def find_max(blobs):
    """ Find maximum blob in a list of blobs
        :input: a list of blobs
        :return: Blob with the maximum area,
                 None if an empty list is passed
    """
    max_blob = None
    max_area = 0
    for blob in blobs:
        if blob.area() > max_area:
            max_blob = blob
            max_area = blob.pixels()
    return max_blob


def comp_new_threshold(statistics, mul_stdev=2):
    """ Generating new thresholds based on detection statistics
        l_low = l_mean - mul_stdev*l_stdev
        l_high = l_mean + mul_stdev*l_stdev
        a_low = a_mean - mul_stdev*a_stdev
        a_high = a_mean - mul_stdev*a_stdev
        b_low = b_mean - mul_stdev*b_stdev
        b_high = b_mean - mul_stdev*b_stdev
    """
    l_mean = statistics.l_mean()
    l_stdev = statistics.l_stdev()
    a_mean = statistics.a_mean()
    a_stdev = statistics.a_stdev()
    b_mean = statistics.b_mean()
    b_stdev = statistics.b_stdev()
    new_threshold = (l_mean - mul_stdev*l_stdev, l_mean + mul_stdev*l_stdev,
                     a_mean - mul_stdev*a_stdev, a_mean - mul_stdev*a_stdev,
                     b_mean - mul_stdev*b_stdev, b_mean - mul_stdev*b_stdev)
    return new_threshold


def comp_weighted_avg(vec1, vec2, w1=0.5, w2=0.5):
    """ Weighted average, by default just normal average
    """
    avg = [int(w1*vec1[i] + w2*vec2[i]) for i in range(len(vec1))]
    return tuple(avg)


class BlobTracker:
    """ BlobTracker class that initializes with a single TrackedBlob and tracks it
        with dynamic threshold
        TODO: track multiple blobs
    """
    def __init__(self, tracked_blob: TrackedBlob, thresholds, clock, show=True):
        self.tracked_blob = tracked_blob
        self.original_thresholds = [threshold for threshold in thresholds]
        self.current_thresholds = [threshold for threshold in thresholds]
        self.clock = clock
        self.show = show

    def track(self):
        """ Detect blobs with tracking capabilities
            :input: tracked_blob: a TrackedBlob class object
                    thresholds: the list of color thresholds we want to track
                    show: True if we want to visualize the tracked blobs
                    clock: clock
        """
        # initialize the blob with the max blob in view if it is not initialized
        if not self.tracked_blob.blob_history:
            reference_blob, statistics = find_reference(self.clock,
                                                        self.original_thresholds,
                                                        time_show_us=0)
            blue_led.on()
            self.tracked_blob.reinit(reference_blob)
            # update the adaptive threshold
            new_threshold = comp_new_threshold(statistics, 2.5)
            for i in range(len(self.current_thresholds)):
                self.current_thresholds[i] = comp_weighted_avg(self.current_thresholds[i],
                                                new_threshold, 1-THRESHOLD_UPDATE_RATE,
                                                THRESHOLD_UPDATE_RATE)

            # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
            return self.tracked_blob.feature_vector, True
        else:
            # O.W. update the blob
            img = sensor.snapshot()
            self.clock.tick()
            blobs = img.find_blobs(self.current_thresholds, merge=True,
                                   pixels_threshold=75,
                                   area_threshold=100,
                                   merge_distance=20)
            blue_led.on()
            roi = self.tracked_blob.update(blobs)
            if self.tracked_blob.untracked_frames >= 15:
                # if the blob fails to track for 15 frames, reset the tracking
                self.tracked_blob.reset()
                blue_led.off()
                print("boom!")
                self.current_thresholds = [threshold for threshold in self.original_thresholds]
                return None, False
            else:
                if roi:
                    statistics = img.get_statistics(roi=roi)
                    new_threshold = comp_new_threshold(statistics, 3.0)
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(self.current_thresholds[i],
                                                        new_threshold, 1-THRESHOLD_UPDATE_RATE,
                                                        THRESHOLD_UPDATE_RATE)
                else:
                    for i in range(len(self.current_thresholds)):
                        self.current_thresholds[i] = comp_weighted_avg(self.original_thresholds[i],
                                                                       self.current_thresholds[i])
                # x, y, z = verbose_tracked_blob(img, tracked_blob, show)
                if self.show:
                    x0, y0, w, h = [math.floor(self.tracked_blob.feature_vector[i]) for i in range(4)]
                    img.draw_rectangle(x0, y0, w, h)
                    st = "FPS: {}".format(str(round(self.clock.fps(),2)))
                    img.draw_string(0, 0, st, color = (255,0,0))
                return self.tracked_blob.feature_vector, True


def verbose_tracked_blob(img, tracked_blob, show):
    """ Converting the tracked blob detection information into relative positions
        from the camera to the blob. We need pre-calibration information for a meaningful
        linear regression. Thus, the function is not used as for now.
    """
    if framesize == sensor.HQVGA:
        x_size = 240
        y_size = 160
    elif framesize == sensor.QQVGA:
        x_size = 160
        y_size = 120
    else:
        assert(False)
    linear_regression_feature_vector = [0 ,0, 0, 0]
    num_blob_history = len(tracked_blob.blob_history)
    for i in range(num_blob_history):
        linear_regression_feature_vector[0] += tracked_blob.blob_history[i].cx()
        linear_regression_feature_vector[1] += tracked_blob.blob_history[i].cy()
        linear_regression_feature_vector[2] += tracked_blob.blob_history[i].w()
        linear_regression_feature_vector[3] += tracked_blob.blob_history[i].h()
    for i in range(4):
        linear_regression_feature_vector[i] /= num_blob_history
    feature_vec = [linear_regression_feature_vector[0]/x_size,
                   linear_regression_feature_vector[1]/y_size,
                   math.sqrt(x_size*y_size/(linear_regression_feature_vector[2]*
                             linear_regression_feature_vector[3]))]
    dist = 0.27485909*feature_vec[2] + 0.9128014726961156
    theta = -0.98059103*feature_vec[0] + 0.5388727340530889
    phi = -0.57751757*feature_vec[1] + 0.24968235246037554
    z = dist*math.sin(phi)
    xy = dist*math.cos(phi)
    x = xy*math.cos(theta)
    y = xy*math.sin(theta)
    if show:
        x0, y0, w, h = [math.floor(tracked_blob.feature_vector[i]) for i in range(4)]
        img.draw_rectangle(x0, y0, w, h)
        st = "FPS: {}".format(str(round(clock.fps(),2)))
        img.draw_string(0, 0, st, color = (255,0,0))
    return x, y, z


def one_norm_dist(v1, v2):
    # 1-norm distance between two vectors
    return sum([abs(v1[i] - v2[i]) for i in range(len(v1))])


def two_norm_dist(v1, v2):
    # 2-norm distance between two vectors
    return math.sqrt(sum([(v1[i] - v2[i])**2 for i in range(len(v1))]))


class TrackedBlob:
    """ TrackedBlob class:
        An advanced class that tracks a colored blob based on a feature vector of 5 values:
            center x, center y, bbox width, bbox height, rotation angle

        It has a window to compute the feature distance for enhanced smoothness
    """
    def __init__(self, init_blob, norm_level: int,
                       feature_dist_threshold=100,
                       window_size = 3, blob_id=0):
        self.blob_history = [init_blob]
        self.feature_vector = [init_blob.x(),
                               init_blob.y(),
                               init_blob.w(),
                               init_blob.h(),
                               init_blob.rotation_deg()]

        self.norm_level = norm_level
        self.untracked_frames = 0
        self.feature_dist_threshold = feature_dist_threshold
        self.window_size = window_size
        self.id = blob_id

    def reset(self):
        """ Reset the tracker by empty the blob_history and feature vector while
            keeping the parameters
        """
        self.blob_history = None
        self.feature_vector = None

    def reinit(self, blob):
        """ reinitialize a reset blob by populate its history list and
            feature vector with a new blob
        """
        self.blob_history = [blob]
        self.feature_vector = [blob.x(),
                               blob.y(),
                               blob.w(),
                               blob.h(),
                               blob.rotation_deg()]
        self.untracked_frames = 0

    def compare(self, new_blob):
        """ Compare a new blob with a tracked blob in terms of
            their feature vector distance
        """
        feature = (new_blob.x(),
                   new_blob.y(),
                   new_blob.w(),
                   new_blob.h(),
                   new_blob.rotation_deg())
        my_feature = self.feature_vector
        if not new_blob.code() == self.blob_history[-1].code():
            # Different colors automatically grant a maimum distance
            return 32767
        elif self.norm_level == 1:
            return (math.fabs(feature[0]-my_feature[0]) +
                    math.fabs(feature[1]-my_feature[1]) +
                    math.fabs(feature[2]-my_feature[2]) +
                    math.fabs(feature[3]-my_feature[3]) +
                    math.fabs(feature[4]-my_feature[4]))
        else:
            return math.sqrt((feature[0]-my_feature[0])**2 +
                             (feature[1]-my_feature[1])**2 +
                             (feature[2]-my_feature[2])**2 +
                             (feature[3]-my_feature[3])**2 +
                             (feature[4]-my_feature[4])**2)


    def update(self, blobs):
        """ Update a tracked blob with a list of new blobs in terms of their feature distance.
            Upon a new candidate blob, we update the tracking history based on whether the
            histroy list is already filled or not
        """
        if blobs is None:
            # auto fail if None is fed
            self.untracked_frames += 1
            return None

        min_dist = 32767
        candidate_blob = None
        for b in blobs:
            # find the blob with minimum feature distance
            dist = self.compare(b)
            if dist < min_dist:
                min_dist = dist
                candidate_blob = b

        if min_dist < self.feature_dist_threshold:
            # update the feature history if the feature distance is below the threshold
            self.untracked_frames = 0
            print("Successful Update! Distance: {}".format(min_dist))
            history_size = len(self.blob_history)
            self.blob_history.append(candidate_blob)
            feature = (candidate_blob.x(),
                       candidate_blob.y(),
                       candidate_blob.w(),
                       candidate_blob.h(),
                       candidate_blob.rotation_deg())

            if history_size <  self.window_size:
                # populate the history list if the number of history blobs is below the
                # window size
                for i in range(5):
                    # calculate the moving average
                    self.feature_vector[i] = (self.feature_vector[i]*history_size +
                        feature[i])/(history_size + 1)
            else:
                # O.W. pop the oldest and push a new one
                oldest_blob = self.blob_history[0]
                oldest_feature = (oldest_blob.x(),
                                  oldest_blob.y(),
                                  oldest_blob.w(),
                                  oldest_blob.h(),
                                  oldest_blob.rotation_deg())
                for i in range(5):
                    self.feature_vector[i] = (self.feature_vector[i]*self.window_size +
                        feature[i] - oldest_feature[i])/self.window_size
                self.blob_history.pop(0)
            return candidate_blob.rect()
        else:
            self.untracked_frames += 1
            return None


def find_reference(clock, thresholds,
                   density_threshold=0.3,
                   roundness_threshold=0.4,
                   time_show_us=50000):
    """ Find a reference blob that is dense and round,
        also return the color statistics in the shrunk bounding box
    """
    biggest_blob = None
    while not biggest_blob:
        blob_list = []
        clock.tick()
        img = sensor.snapshot()
        b_blobs = img.find_blobs(thresholds, merge=True,
                                 pixels_threshold=75,
                                 area_threshold=100,
                                 merge_distance=20)
        for blob in b_blobs:
            # find a good initial blob by filtering out the not-so-dense and not-so-round blobs
            if (blob.density() > density_threshold and
                blob.roundness() > roundness_threshold):
                blob_list.append(blob)

        biggest_blob = find_max(blob_list)

    draw_initial_blob(img, biggest_blob, time_show_us)
    statistics = img.get_statistics(roi=biggest_blob.rect())
    return biggest_blob, statistics


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

def send_blob_message(arr, initial= 0):
    pass



if __name__ == "__main__":
    led_pin = Pin("PG12", Pin.OUT)
    led_pin.value(0)
    blue_led = pyb.LED(3)
    clock = time.clock()
    # Sensor initialization
    init_sensor_target()
    # Initialize ToF sensor
    # tof = VL53L1X(I2C(2))
    # Initialize UART
    uart = UART("LP1", 115200, timeout_char=2000) # (TX, RX) = (P1, P0) = (PB14, PB15)
    # Find reference
    thresholds = PURPLE
    reference_blob, statistics = find_reference(clock, thresholds)
    tracked_blob = TrackedBlob(reference_blob, norm_level=1, feature_dist_threshold=100)
    blob_tracker = BlobTracker(tracked_blob, thresholds, clock)


    while True:
        blob_tracker.track()
        msg = bytearray(32)
        msg[0] = 0x20
        msg[1] = 0x40
        # x y w h
        if blob_tracker.tracked_blob.feature_vector:
            x_value = int(blob_tracker.tracked_blob.feature_vector[0])
            y_value = int(blob_tracker.tracked_blob.feature_vector[1])
            w_value = int(blob_tracker.tracked_blob.feature_vector[2])
            h_value = int(blob_tracker.tracked_blob.feature_vector[3])
            cx_msg = bytearray(x_value.to_bytes(2, 'little'))
            msg[2] = cx_msg[0]
            msg[3] = cx_msg[1]
            cy_msg = bytearray(y_value.to_bytes(2, 'little'))
            msg[4] = cy_msg[0]
            msg[5] = cy_msg[1]
            w_msg = bytearray(w_value.to_bytes(2, 'little'))
            msg[6] = w_msg[0]
            msg[7] = w_msg[1]
            h_msg = bytearray(h_value.to_bytes(2, 'little'))
            msg[8] = h_msg[0]
            msg[9] = h_msg[1]
        else:
            msg[2] = 0x0
            msg[3] = 0x0
            msg[4] = 0x0
            msg[5] = 0x0
            msg[6] = 0x0
            msg[7] = 0x0
            msg[8] = 0x0
            msg[9] = 0x0
        # distance
        try: dis = 9999 # tof.read()
        except: dis = 9999
        dis_msg = bytearray(dis.to_bytes(2, 'little'))
        msg[10] = dis_msg[0]
        msg[11] = dis_msg[1]
        # Perform the checksume
        chA, chB = checksum(msg[:-2], 0)
        msg[-1] = chA
        msg[-2] = chB

        uart.write(msg)         # send 32 byte message
