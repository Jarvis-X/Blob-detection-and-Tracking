# By: JiaweiXuAirlab - Wed Apr 3 2024

import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_framesize(sensor.HQVGA)

sensor.set_auto_whitebal(True)
sensor.set_auto_exposure(True)
#sensor.set_auto_gain(True)
sensor.skip_frames(time = 2000)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
sensor.skip_frames(time = 2000)


sensor.__write_reg(0xfe, 0) # change to registers at page 0
sensor.__write_reg(0x80, 0b01111111)    # [7] reserved, [6] gamma enable, [5] CC enable,
                                        # [4] Edge enhancement enable
                                        # [3] Interpolation enable, [2] DN enable, [1] DD enable,
                                        # [0] Lens-shading correction enable
sensor.__write_reg(0x81, 0b11011100)    # [7] BLK dither mode, [6] low light Y stretch enable
                                        # [5] skin detection enable, [4] reserved, [3] new skin mode
                                        # [2] autogray enable, [1] reserved, [0] BFF test image mode
sensor.__write_reg(0x82, 0b00000100)    # [2] ABS enable, [1] AWB enable
sensor.__write_reg(0x90, 0b00000110)    # disable Neighbor average and enable chroma correction

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

# color correction -- this is very tricky: the color shifts on the color wheel it seems
#sensor.__write_reg(0xfe, 2) # change to registers at page 2
# WARNING: uncomment the two lines to invert the color
#sensor.__write_reg(0xc1, 0x80)          # CC_CT1_11, feels like elements in a matrix
#sensor.__write_reg(0xc5, 0x80)          # CC_CT1_22 , feels like elements in a matrix

# ABS - anti-blur
sensor.__write_reg(0xfe, 1)             # change to registers at page 1
sensor.__write_reg(0x9a, 0b00000111)    # [7:4] add dynamic range, [2:0] abs adjust every frame
sensor.__write_reg(0x9d, 0xff)          # [7:0] Y stretch limit


""" What are the auto values
"""
sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
high_exp = sensor.__read_reg(0x03)  # high bits of exposure control
low_exp = sensor.__read_reg(0x04)   # low bits of exposure control
print("high expo:\t\t", high_exp)
print("low expo:\t\t", low_exp)
print("global gain:\t\t", sensor.__read_reg(0xb0))   # global gain

# RGB gains
R_gain = sensor.__read_reg(0xb3)
G_gain = sensor.__read_reg(0xb4)
B_gain = sensor.__read_reg(0xb5)
pre_gain = sensor.__read_reg(0xb1)
pos_gain = sensor.__read_reg(0xb2)

print("R gain:\t\t", R_gain)    # R auto gain
print("G gain:\t\t", G_gain)    # G auto gain
print("B gain:\t\t", B_gain)    # B auto gain
print("pre-gain:\t\t", pre_gain)    # auto pre-gain, whatever that means
print("post-gain:\t\t", pos_gain)   # auto post-gain, whatever that means


sensor.__write_reg(0xfe, 0b00000010)    # change to registers at page 2
print("Global saturation:\t", sensor.__read_reg(0xd0))  # change global saturation,
                                                        # strangely constrained by auto saturation
print("Cb saturation:\t\t", sensor.__read_reg(0xd1))  # Cb saturation
print("Cr saturation:\t\t", sensor.__read_reg(0xd2))  # Cr saturation
print("luma contrast:\t\t", sensor.__read_reg(0xd3))  # luma contrast
print("luma offset:\t\t", sensor.__read_reg(0xd5))    # luma offset
print("showing off")
for i in range(50):
    img = sensor.snapshot()



""" Set the values - be aware, some of them are quite tricky
    as some registers do not give a **** to what we write
    and the related settings are actually controlled by some
    other registers
"""
print("setting up")
sensor.reset()
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
sensor.set_pixformat(sensor.RGB565)
sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
sensor.set_framesize(sensor.HQVGA)
sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
sensor.__write_reg(0xb0, 0b10000000) # global gain
sensor.__write_reg(0xad, (R_gain << 0)) # R gain ratio
sensor.__write_reg(0xae, (G_gain << 0)) # G gain ratio
sensor.__write_reg(0xaf, (B_gain << 0)) # B gain ratio
sensor.__write_reg(0x03, high_exp)  # high bits of exposure control
sensor.__write_reg(0x04, low_exp)   # low bits of exposure control


while(True):
    img = sensor.snapshot()
