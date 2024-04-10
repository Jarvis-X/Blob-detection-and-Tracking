# This work is licensed under the MIT license.
# Copyright (c) 2013-2023 OpenMV LLC. All rights reserved.
# https://github.com/openmv/openmv/blob/master/LICENSE
#
# Color Binary Filter Example
#
# This script shows off the binary image filter. You may pass binary any
# number of thresholds to segment the image by.

import sensor
import time
import math
import random
from lib.Ibus import IBus
from pyb import UART

sensor.reset()
sensor.set_framesize(sensor.HQVGA)
sensor.set_pixformat(sensor.RGB565)
sensor.skip_frames(time=2000)
clock = time.clock()

old_metric = 0

lo, hi = -5, 5
red_threshold = [40, 80, 0, 12, None, None]  # [0, 100, -15, 15, 14, 78] (0, 100, 2, 18, -15, -6)
old_loc = (0, 0)

# Color
cb = -10
MIN_CB = cb - 10
MAX_CB = cb + 10

nfc = 0  # not found counter
blackandwhite = True

CB_ALTERNATIVES = [MIN_CB, cb, MAX_CB]
ACTIONS = (-.5, -.3, .3, .5)
# ACTIONS=[0]



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
        ones = []
        # Loop through each cell of the grid
        for row in range(self.num_rows):
            for col in range(self.num_cols):
                # Define the region of interest (ROI) for the current cell
                roi = (col * self.cell_width, row * self.cell_height, self.cell_width, self.cell_height)
                hist = img.get_histogram(bins=2, roi=roi)

                # Count the number of white pixels (ones) within the current cell
                ones_in_cell = hist.bins()[1] * self.cell_width * self.cell_height
                ones.append(ones_in_cell)
                # print(f"Number of ones (white pixels) in cell ({row},{col}):", ones_in_cell)

                # Draw the number of ones in the corner of the cell
                img.draw_string(roi[0], roi[1], str(int(ones_in_cell)), color=(255))  #

                # Draw the ROI on the image
                img.draw_rectangle(roi, color=(int(ones_in_cell * 255)), thickness=1)
        return ones

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

    def action(self,nones):
        # Counting
        left_cells = self.count_column(nones, 0)
        right_cells = self.count_column(nones, 2)
        up_cells = self.count_row(nones, 0)
        down_cells = self.count_row(nones, 2)

        # Control action
        # print(left_cells, right_cells, up_cells, down_cells)
        ux = int((right_cells - left_cells) * img.width()) // 3
        uy = -int((up_cells - down_cells) * img.height()) // 3

        return ux, uy








print("Start")

if __name__ == "__main__":

    img = sensor.snapshot()
    detector = GridDetector(3, 3, img.width(), img.height())

    # Initialize UART
    uart = UART("LP1", 115200, timeout_char=2000)  # (TX, RX) = (P1, P0) = (PB14, PB15)

    while True:
        clock.tick()

        img = sensor.snapshot()


#        # Apply random action
        action = _normal_dist(sigma=2)
        cb += action
        cb = min(MAX_CB, max(MIN_CB, cb))

        # B interval
        red_threshold[4] = int(cb) - hi
        red_threshold[5] = int(cb) + hi

        if blackandwhite:
            img = img.binary([red_threshold])
            # Convert the binary image to grayscale
            img = img.to_grayscale()
            threshold = (0, 255)
        else:
            img = img.binary([red_threshold])
            threshold = red_threshold



        ones = detector.count(img)
        nones = detector.normalize(ones)
        mean, variance = detector.statistics(nones)


        metric = variance

        if metric < old_metric:
            cb -= action

        old_metric = metric


        if sum(ones) > 50:
            nfc = 0
            print(cb, metric, action, sum(ones), int(clock.fps()))
        else:
            nfc += 1
            if nfc == 10:
                cb = random.choice(CB_ALTERNATIVES)
                print("reset", cb)
                nfc = 0
                old_metric = -1000


        ux, uy = detector.action(nones)

        x0 = img.width() // 2
        y0 = img.height() // 2
        img.draw_arrow(x0, y0, x0 + ux, y0 + uy, color=(0, 255, 0), size=30, thickness=2)



        # Create message for ESP32
        flag = 0b10000000
        x_roi,y_roi = ux, uy
        w_roi, h_roi = 10,10
        x_value,y_value = ux, uy
        w_value = 10
        h_value = 10
        dis = 9999  # Time of flight sensor fixme

        msg = IBus_message([flag, x_roi, y_roi, w_roi, h_roi,
                            x_value, y_value, w_value, h_value, dis])


        # Send message
        uart.write(msg)

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
