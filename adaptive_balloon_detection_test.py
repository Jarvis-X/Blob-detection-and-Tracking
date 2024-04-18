# Untitled - By: JiaweiXuAirlab - Wed Apr 3 2024

import sensor, image, time, random, math

sensor.reset()
sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
sensor.set_pixformat(sensor.RGB565)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
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
sensor.__write_reg(0xc1, 0x80)          # CC_CT1_11, feels like elements in a matrix
sensor.__write_reg(0xc5, 0x80)          # CC_CT1_22 , feels like elements in a matrix
print("color correction setup done")

# ABS - anti-blur
sensor.__write_reg(0xfe, 1)             # change to registers at page 1
sensor.__write_reg(0x9a, 0b00000111)    # [7:4] add dynamic range, [2:0] abs adjust every frame
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
sensor.__write_reg(0xad, 64)    # R gain ratio
sensor.__write_reg(0xae, 64)    # G gain ratio
sensor.__write_reg(0xaf, 92)    # B gain ratio
sensor.set_auto_exposure(True)
sensor.skip_frames(time = 1000)
print("AWB Gain setup done.")


# color setup - saturation
sensor.__write_reg(0xfe, 2)     # change to registers at page 2
sensor.__write_reg(0xd0, 128)    # change global saturation,
sensor.__write_reg(0xd1, 48)    # Cb saturation
sensor.__write_reg(0xd2, 48)    # Cr saturation
sensor.__write_reg(0xd3, 64)    # contrast
sensor.__write_reg(0xd5, 0)     # luma offset


clock = time.clock()


class Adaptive_FindBlob:
    def __init__(self, initial_threshold=[0, 100, -120, 120, -120, 120],
                 threshold_AB_bounds=[-120, 120, -120, 120],
                 initial_step_size=2, explore_rate = 2, explore_ratio = 0.1):
        self.threshold = initial_threshold
        self.threshold_AB_bounds = threshold_AB_bounds
        self.update_step = initial_step_size
        self.explore_ratio = explore_ratio
        self.explore_rate = explore_rate
        self.value = 0
        self.AB_lowhigh_update = [0, 0, 0, 0]


    def detect(self, img, invert=False, x_stride=1, y_stride=1,
               area_threshold=10, pixels_threshold=10, merge=True, margin=1):
        blobs = img.find_blobs([self.threshold], invert=invert,
                               x_stride=x_stride, y_stride=y_stride,
                               area_threshold=area_threshold, pixels_threshold=pixels_threshold,
                               merge=merge, margin=margin)

        max_blob = self._find_max(blobs)
        if max_blob:
            self.update_threshold(max_blob)
        return max_blob


    def update_threshold(self, blob):
        value = (blob.density() +
                 blob.compactness() +
                 blob.solidity() +
                 blob.roundness()) / 2.0

        value_diff = value - self.value
        for i in range(4):
            self.AB_lowhigh_update[i] = (
                1 - self.explore_ratio
            ) * self.AB_lowhigh_update[i] * value_diff +\
            self.explore_ratio * self.explore_rate * (2*random.random() - 1)
            self.threshold[i+2] += self.AB_lowhigh_update[i]
            if i < 2:
                if self.threshold[i+2] < self.threshold_AB_bounds[0]:
                    self.threshold[i+2] = self.threshold_AB_bounds[0]
                if self.threshold[i+2] > self.threshold_AB_bounds[1]:
                    self.threshold[i+2] = self.threshold_AB_bounds[1]
            else:
                if self.threshold[i+2] < self.threshold_AB_bounds[2]:
                    self.threshold[i+2] = self.threshold_AB_bounds[2]
                if self.threshold[i+2] > self.threshold_AB_bounds[3]:
                    self.threshold[i+2] = self.threshold_AB_bounds[3]
        print(self.threshold)

    def _find_max(self, blobs: list) -> image.blob:
        """
        @description: Find the blob with the largest area
        @param       {list} nice_blobs: The list of blobs to be compared
        @return      {image.blob} The blob with the largest area
        """
        max_blob = None
        max_area = 0
        for blob in blobs:
            if blob.area() > max_area:
                max_blob = blob
                max_area = blob.pixels()
        return max_blob


while(True):
    clock.tick()
    img = sensor.snapshot()
    adaptive_blob_finder = Adaptive_FindBlob(initial_threshold=[0, 100, -25, -5, 25, 60],
                                             threshold_AB_bounds=[-35, 10, 20, 60],
                                             initial_step_size=5.0, explore_rate = 10.0, explore_ratio = 0.3)
    blob = adaptive_blob_finder.detect(img)
    if blob:
        img.draw_edges(blob.min_corners(), color=(255, 0, 0))
        img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
        img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
        img.draw_rectangle(blob.rect())
        img.draw_cross(blob.cx(), blob.cy())
        img.draw_keypoints([(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20)

