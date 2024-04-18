from lib.Ibus import IBus
import time
import sensor
from lib.tracker import BLOBTracker, GoalTracker
from pyb import UART
from machine import I2C
from vl53l1x import VL53L1X
#from lib.Ibus import IBus

frame_rate = 80
def blob_tracking(thresholds, clock, blob_type=1, isColored=True):
    """ The blob tracker initialization for balloons
        blob_type=1 for balloons
        blob_type=2 for goals
    """
    if blob_type == 1:
        blob_tracker = BLOBTracker(thresholds, clock)
    elif blob_type == 2:
        blob_tracker = GoalTracker(thresholds, clock, sensor_sleep_time=WAIT_TIME_US)
    else:
        raise ValueError("Invalid blob type!")
    return blob_tracker


def init_sensor_target(tracking_type:int=0, isColored:bool=True,
                       framesize=sensor.HQVGA, windowsize=None) -> None:
    """ Initialize sensors by updating the registers
        for the two different purposes
        @param       {int} tracking_type: 0 for balloons and 1 for goals
        @param       {bool} isColored: whether we would like to initialize the sensor
                                       in colored mode
        @return      {*} None
    """
    # We do these whatever mode we are in
    sensor.reset()
#    sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
#    sensor.set_auto_whitebal(False)
    sensor.set_auto_exposure(False)
#    sensor.__write_reg(0xfe, 0b00000000) # change to registers at page 0
#    sensor.__write_reg(0x80, 0b10111100) # enable gamma, CC, edge enhancer, interpolation, de-noise
#    sensor.__write_reg(0x81, 0b01111100) # enable BLK dither mode, low light Y stretch, autogray enable
#    sensor.__write_reg(0x82, 0b00000100) # enable anti blur, disable AWB
    sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
    sensor.__write_reg(0x80, 0b01111110)    # [7] reserved, [6] gamma enable, [5] CC enable,
    sensor.__write_reg(0x03, 0b00000010) # high bits of exposure control
    sensor.__write_reg(0x04, 0b11000000) # low bits of exposure control


#    # color setup - saturation
#    sensor.__write_reg(0xfe, 2)     # change to registers at page 2
#    sensor.__write_reg(0xd0, 128)    # change global saturation,
#    sensor.__write_reg(0xd1, 48)    # Cb saturation
#    sensor.__write_reg(0xd2, 48)    # Cr saturation
#    sensor.__write_reg(0xd3, 64)    # contrast
#    sensor.__write_reg(0xd5, 0)     # luma offset

#    sensor.__write_reg(0xb0, 0b01100000) # global gain
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(framesize)
#    if windowsize is not None:
#        sensor.set_windowing(windowsize)

#    # change sensor color type and exposure based on the tracking type
#    if isColored and tracking_type == 0:
#        # For balloon tracking
#        """ Night """
#        sensor.set_pixformat(sensor.RGB565)
#        sensor.__write_reg(0x90, 0b00000110) # disable Neighbor average and enable chroma correction
#        sensor.__write_reg(0x03, 0b00000000) # high bits of exposure control
#        sensor.__write_reg(0x04, 0b11000000) # low bits of exposure control
#        sensor.__write_reg(0xb0, 0b11000000) # global gain
#        # RGB gains
#        sensor.__write_reg(0xa3, 0b10001000) # G gain odd
#        sensor.__write_reg(0xa4, 0b10001000) # G gain even
#        sensor.__write_reg(0xa5, 0b10000010) # R gain odd
#        sensor.__write_reg(0xa6, 0b10000010) # R gain even
#        sensor.__write_reg(0xa7, 0b11001100) # B gain odd
#        sensor.__write_reg(0xa8, 0b11001100) # B gain even
#        sensor.__write_reg(0xa9, 0b10001000) # G gain odd 2
#        sensor.__write_reg(0xaa, 0b10001000) # G gain even 2
#        sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
#        # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
#                                               # strangely constrained by auto saturation
#        sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
#        sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
#        sensor.__write_reg(0xd3, 0b00101000) # luma contrast
#        # sensor.__write_reg(0xd5, 0b00000000) # luma offset

#    elif isColored and tracking_type == 1:
#        # For target tracking, colored
#        sensor.reset()
#        sensor.ioctl(sensor.IOCTL_SET_FOV_WIDE, True)
#        sensor.set_auto_whitebal(False)
#        sensor.set_auto_exposure(False)
#        sensor.set_pixformat(sensor.RGB565)
#        sensor.__write_reg(0x90, 0b00000110) # disable Neighbor average and enable chroma correction
#        sensor.set_framesize(framesize)
##        sensor.set_framerate(frame_rate)
#        sensor.skip_frames(10)
#        if False:# used for off on off
#            sensor.__write_reg(0x03, 0b00000010) # high bits of exposure control
#            sensor.__write_reg(0x04, 0b11000000) # low bits of exposure control
#            sensor.__write_reg(0xb0, 0b01100000) # global gain
#        elif True:# used for on off on #small blue on camera
#            sensor.__write_reg(0x03, 0b0000001) # high bits of exposure control
#            sensor.__write_reg(0x04, 0b10110000) # low bits of exposure control
#            sensor.__write_reg(0xb0, 0b01110000) # global gain

#        elif False:# used for on off on in highbay during day?
#            sensor.__write_reg(0x03, 0b00000001) # high bits of exposure control
#            sensor.__write_reg(0x04, 0b10000000) # low bits of exposure control
#            sensor.__write_reg(0xb0, 0b01110000) # global gain
##        # RGB gains
##        sensor.__write_reg(0xa3, 0b10000000) # G gain odd
##        sensor.__write_reg(0xa4, 0b10000000) # G gain even
##        sensor.__write_reg(0xa5, 0b01101000) # R gain odd
##        sensor.__write_reg(0xa6, 0b01101000) # R gain even
##        sensor.__write_reg(0xa7, 0b01110100) # B gain odd
##        sensor.__write_reg(0xa8, 0b01110100) # B gain even
##        sensor.__write_reg(0xa9, 0b10000000) # G gain odd 2
##        sensor.__write_reg(0xaa, 0b10000000) # G gain even 2
##        sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
##        # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
##                                               # strangely constrained by auto saturation
##        sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
##        sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
##        sensor.__write_reg(0xd3, 0b01000000) # luma contrast
##        # sensor.__write_reg(0xd5, 0b00000000) # luma offset
#        sensor.skip_frames(10)
#    elif tracking_type == 1:
#        # For target tracking, BnW
#        sensor.set_pixformat(sensor.GRAYSCALE)
#        sensor.__write_reg(0x90, 0b00000110) # disable Neighbor average and enable chroma correction
#        sensor.set_framesize(framesize)
#        sensor.set_framerate(frame_rate)
#        sensor.skip_frames(10)
#        sensor.__write_reg(0x03, 0b00000000) # high bits of exposure control
#        sensor.__write_reg(0x04, 0b01111000) # low bits of exposure control
#        sensor.__write_reg(0xb0, 0b10000000) # global gain
##        # RGB gains
##        sensor.__write_reg(0xa3, 0b10000000) # G gain odd
##        sensor.__write_reg(0xa4, 0b10000000) # G gain even
##        sensor.__write_reg(0xa5, 0b01101000) # R gain odd
##        sensor.__write_reg(0xa6, 0b01101000) # R gain even
##        sensor.__write_reg(0xa7, 0b01110100) # B gain odd
##        sensor.__write_reg(0xa8, 0b01110100) # B gain even
##        sensor.__write_reg(0xa9, 0b10000000) # G gain odd 2
##        sensor.__write_reg(0xaa, 0b10000000) # G gain even 2
##        sensor.__write_reg(0xfe, 0b00000010) # change to registers at page 2
##        # sensor.__write_reg(0xd0, 0b00000000) # change global saturation,
##                                               # strangely constrained by auto saturation
##        sensor.__write_reg(0xd1, 0b01000000) # change Cb saturation
##        sensor.__write_reg(0xd2, 0b01000000) # change Cr saturation
##        sensor.__write_reg(0xd3, 0b01000000) # luma contrast
##        # sensor.__write_reg(0xd5, 0b00000000) # luma offset
#    else:
#        raise ValueError("Not a valid sensor-detection mode!")

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


def mode_initialization(input_mode, mode, isColored):
    """ Switching between blinking goal tracker and balloon tracker
    """
    if mode == input_mode:
        print("already in the mode")
        return None
    else:
        if input_mode == 0:
            # balloon tracking mode
            init_sensor_target(tracking_type=0, isColored=isColored)
            thresholds = PURPLE + GREEN
            tracker = blob_tracking(thresholds, clock, blob_type=1)
            print("balloon mode!")
        elif input_mode == 1 and not isColored:
            init_sensor_target(tracking_type=1, isColored=isColored)
            # Find reference
            thresholds = GRAY
            tracker = blob_tracking(thresholds, clock, blob_type=2)
            print("BnW goal mode!")
        elif input_mode == 1 and isColored:
            init_sensor_target(tracking_type=1, isColored=isColored)
            # Find reference
            thresholds = TARGET_COLOR
            tracker = blob_tracking(thresholds, clock, blob_type=2)
            print("colored goal mode!")

        return input_mode, tracker


if __name__ == "__main__":
    ### Macros
    """ Day
    GREEN = [(42, 50, -20, -5, -31, -8)]
    PURPLE = [(41, 52, 25, 54, -78, -52)] """

    """ Auto color Day
    PURPLE = [(17, 48, 9, 33, -21, 0)]
    """
    """ Auto color Night
    PURPLE = [(22, 32, 7, 17, -24, -10)] """

    """ Night """
    GREEN = [(34, 46, -32, -3, -20, 14)]
    PURPLE = [(24, 35, 4, 22, -31, -10)]

    GRAY = [(0, 200)]
    ORANGE_TARGET = [(55, 100, -12, 13, 27, 54)]
    TARGET_COLOR = [(0, 100, -28, 10, 17, 62)]#[(48, 100, -44, -14, 30, 61)]#[(54, 100, -56, -5, 11, 70), (0, 100, -78, -19, 23, 61)]#[(49, 97, -45, -6, -16, 60),(39, 56, -12, 15, 48, 63), (39, 61, -19, 1, 45, 64), (20, 61, -34, 57, -25, 57)] # orange, green
    THRESHOLD_UPDATE_RATE = 0.0
    WAIT_TIME_US = 1000000//frame_rate
    ### End Macros

    clock = time.clock()
    ISCOLORED = True
    mode = 1

    # Initialize inter-board communication
    # ibus = IBus()
    # Sensor initialization
    tof = VL53L1X(I2C(2))

    mode, tracker = mode_initialization(mode, -1, ISCOLORED)
    # Initialize UART
    uart = UART("LP1", 115200, timeout_char=2000) # (TX, RX) = (P1, P0) = (PB14, PB15)

    while True:
        feature_vector, flag = tracker.track()
        try: dis = tof.read()
        except: dis = 9999
        if flag & 0b10000000:
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
            msg = IBus_message([flag, 0, 0, 0, 0,
                                0, 0, 0, 0, dis])
        # print(hex(flag))

        uart.write(msg)
        if uart.any():
            uart_input = uart.read()
            print(uart_input)
            if uart_input == b'\x40' and mode == 1:
                ISCOLORED = True
                res = mode_initialization(0, mode, ISCOLORED)
                if res:
                    mode, tracker = res
            elif uart_input == b'\x80' and mode == 0:
                ISCOLORED = False
                res = mode_initialization(1, mode, ISCOLORED)
                if res:
                    mode, tracker = res
