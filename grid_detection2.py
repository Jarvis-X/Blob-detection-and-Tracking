# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Color Binary Filter Example
#
# This script shows off the binary image filter. You may pass binary any
# number of thresholds to segment the image by.

import sensor, image
import time
import math
import random
from lib.Ibus import IBus
from pyb import UART

sensor.reset()
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.skip_frames(time=2000)



clock = time.clock()


#sensor.__write_reg(0xfe, 0) # change to registers at page 0
#sensor.__write_reg(0x80, 0b01111110)    # [7] reserved, [6] gamma enable, [5] CC enable,
#                                        # [4] Edge enhancement enable
#                                        # [3] Interpolation enable, [2] DN enable, [1] DD enable,
#                                        # [0] Lens-shading correction enable - gives you uneven
#                                        #                                      shade in the dark
#                                        #                                      badly!!!!!
#sensor.__write_reg(0x81, 0b01010100)    # [7] BLK dither mode, [6] low light Y stretch enable
#                                        # [5] skin detection enable, [4] reserved, [3] new skin mode
#                                        # [2] autogray enable, [1] reserved, [0] BFF test image mode
#sensor.__write_reg(0x82, 0b00000100)    # [2] ABS enable, [1] AWB enable
##sensor.__write_reg(0x87, 0b00000001)    # [0] auto_edge_effect
#sensor.__write_reg(0x9a, 0b00001111)    # [3] smooth Y, [2] smooth Chroma,
#                                        # [1] neighbor average mode, [0] subsample extend opclk
### color setup - saturation
#sensor.__write_reg(0xfe, 2)     # change to registers at page 2
#sensor.__write_reg(0xd0, 200)    # change global saturation,
#sensor.__write_reg(0xd1, 48)    # Cb saturation
#sensor.__write_reg(0xd2, 48)    # Cr saturation
#sensor.__write_reg(0xd3, 64)    # contrast
#sensor.__write_reg(0xd5, 0)     # luma offset


nfc = 0  # not found counter
blackandwhite = True



N_BINS=50


def distance_point_to_segment(point, segment):
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


def _normal_dist(mu=0, sigma=1):
    u1 = random.uniform(0, 1)
    u2 = random.uniform(0, 1)
    return mu + sigma*math.sqrt(-2.0 * math.log(u1)) * math.cos(2 * math.pi * u2)

class GridDetector:
    def __init__(self, num_rows, num_cols, img_width, img_height):
        self.num_rows = num_rows
        self.num_cols = num_cols
        self.cell_width = int(img_width / num_cols)
        self.cell_height = int(img_height / num_rows)

    def count(self, img):
        metrics = []
        # Loop through each cell of the grid
        for row in range(self.num_rows):
            for col in range(self.num_cols):
                # Define the region of interest (ROI) for the current cell
                roi = (col * self.cell_width, row * self.cell_height, self.cell_width, self.cell_height)
#                hist = img.get_histogram(bins=N_BINS, roi=roi)
#                a_bins = hist.a_bins()
#                b_bins = hist.b_bins()

#                mean1 = sum(a_bins)
                #print(mean1)


                # Copy the ROI from the original image
                stats = img.copy(roi=roi).get_statistics()
##                # Calculate the mean and variance of the ROI
                a_mean = stats.a_mean()
                b_mean = stats.b_mean()
                a_std = stats.a_stdev()
                b_std = stats.b_stdev()


                ### metric ####
                # Distance to the line segment
                mean_ref = (5,-6)
                std_ref=(4, 5)
                d_mean = math.sqrt((mean_ref[0]-a_mean)**2 + (mean_ref[1]-b_mean)**2)
                d_std = math.sqrt((std_ref[0]-a_std)**2 + (std_ref[1]-b_std)**2)


                MAX_DIST=10
                MAX_STD=(8,8)

                # Clamp max distance
                d_mean=d_mean if d_mean<MAX_DIST else MAX_DIST


                metric = 1 - d_mean/MAX_DIST

                if a_std>MAX_STD[0] or b_std>MAX_STD[1]:
                    metric=0

                metrics.append(metric)
                print(metric, a_mean, b_mean, a_std, b_std)
                #variance = img.stdev(roi=roi)


                #print(mean) #"a_bins"
                # Count the number of white pixels (ones) within the current cell
                #ones_in_cell = hist.bins()[1] * self.cell_width * self.cell_height
                #ones.append(ones_in_cell)
                # print(f"Number of ones (white pixels) in cell ({row},{col}):", ones_in_cell)

                # Draw the number of ones in the corner of the cell
                img.draw_string(roi[0], roi[1],"{:.1f}".format(metric) , color=(255))  #

                # Draw the ROI on the image
                img.draw_rectangle(roi, color=(int(metric*2*255)), thickness=1)
        return metrics

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

    def action(self, metric):
        # Counting
        left_cells = sum([self.count_column(metric, i) for i in range(N_COLS//2) ])
        right_cells = sum([self.count_column(metric, N_COLS-i-1) for i in range(N_COLS//2) ])
        up_cells =  sum([self.count_row(metric, i) for i in range(N_ROWS//2) ])
        down_cells = sum([self.count_row(metric, N_ROWS-i-1) for i in range(N_ROWS//2) ])

        # Control action
        # print(left_cells, right_cells, up_cells, down_cells)
        ux = int((right_cells - left_cells) * img.width()) // 3
        uy = -int((up_cells - down_cells) * img.height()) // 3

        return ux, uy








print("Start")

N_ROWS = 3
N_COLS = 3
if __name__ == "__main__":

    img = sensor.snapshot()
    detector = GridDetector(N_ROWS, N_COLS, img.width(), img.height())

    # Initialize UART
#    uart = UART("LP1", 115200, timeout_char=2000)  # (TX, RX) = (P1, P0) = (PB14, PB15)

    while True:
        clock.tick()

        img = sensor.snapshot()
        # Load the image from flash memory
#        img_from_flash = image.Image("bw.jpg")  # Load the image using its filename
        #mutable_img = img_from_flash.copy()

#        sensor.snapshot().copy(mutable_img)
#        mutable_img.get_histogram()

        metric_grid = detector.count(img)




#        nones = detector.normalize(ones)
#        mean, variance = detector.statistics(nones)


#        metric = variance

#        if metric < old_metric:
#            cb -= action

#        old_metric = metric


#        if sum(ones) > 50:
#            nfc = 0
#            print(cb, metric, action, sum(ones), int(clock.fps()))
#        else:
#            nfc += 1
#            if nfc == 10:
#                cb = random.choice(CB_ALTERNATIVES)
#                print("reset", cb)
#                nfc = 0
#                old_metric = -1000


        ux, uy = detector.action(metric_grid)

        x0 = img.width() // 2
        y0 = img.height() // 2
        img.draw_arrow(x0, y0, x0 + ux, y0 + uy, color=(0, 255, 0), size=30, thickness=2)

        print((clock.fps()))

#        # Create message for ESP32
#        flag = 0b10000000
#        x_roi,y_roi = ux, uy
#        w_roi, h_roi = 10,10
#        x_value,y_value = ux, uy
#        w_value = 10
#        h_value = 10
#        dis = 9999  # Time of flight sensor fixme

#        msg = IBus_message([flag, x_roi, y_roi, w_roi, h_roi,
#                            x_value, y_value, w_value, h_value, dis])


        # Send message
#        uart.write(msg)

#        print(clock.fps(), detector.num_rows)
        # Receive message
        # if uart.any():
        #     uart_input = uart.read()
        #     print(uart_input)
        #     if uart_input == b'\x80' and mode == 1:
        #         ISCOLORED = True
        #         res = mode_initialization(0, mode, ISCOLORED)
        #         if res:
        #             mode, tracker = res
        #     elif uart_input == b'\x81' and mode == 0:
        #         ISCOLORED = False
        #         res = mode_initialization(1, mode, ISCOLORED)
        #         if res:
        #             mode, tracker = res
