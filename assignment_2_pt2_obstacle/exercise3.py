from robot import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [

    # Pranathi's bedroom floor - gain 10 - lamp + phone
    # (34, 43, 14, 38, 0, 32), # red 1
    # (39, 56, 16, 38, -6, 20), # red 2
    # # (47, 57, -23, -7, -20, -3), # light blue 1 LEFT
    # # (40, 56, -14, 0, -26, -13), # light blue 2 LEFT
    # (54, 71, -14, 1, -25, -15), # light blue 3 LEFT
    # # (8, 23, -2, 18, -18, 1), # purple 1 RIGHT
    # (13, 28, 8, 25, -25, -7), # purple 2 RIGHT
    # # (9, 42, -11, 5, 13, 41) # obstacle yellow 1
    # (16, 47, -5, 12, 14, 37), # obstacle yellow 2

    #Rachael's Skempton thresholds
    # (41, 46, 27, 42, 7, 35), # red
    # (42, 54, -20, -1, -29, -6), # bluex
    # (19, 29, -8, 31, -22, 13), # purple

    # Pranathi's thresholds in RSM 301C room next to it
    # (52, 62, 21, 47, 5, 18), # red 1
    # (42, 51, 27, 44, 2, 33), # red 2
    # (53, 82, -20, -3, -27, -10), # light blue 1
    # (42, 49, 12, 28, -28, -8), # purple 1
    # # (57, 89, -11, 7, 11, 45), # obstacle yellow 1
    # # (51, 87, -17, 2, 4, 33) # obstacle yellow 2
    # (2, 15, -14, 0, -4, 11) # obstacle black 1

    # Pranathi's living room table gain 10 - lamp, phone
    (54, 76, 16, 46, -14, 20), # red
    (0, 0, 0, 0, 0, 0), # light blue 1
    (0, 0, 0, 0, 0, 0), # purple 1
    (4, 11, -14, -1, -4, 12), # obstacle

]


robot = Robot(thresholds, gain = 10, p_steer=1.1, d_steer=0.005, p=1.2, i=0, d=0.005, imax=0.0)


# DISABLE FRAME BUFFER

# robot.stage1()
# robot.stage1_robust()

# robot.stage2()
# robot.stage2_robust()

# stops = [10, 12, 15, 18, 20, 22, 25, 28, 30]
# for stop in stops:
#     robot.stage3(stop_cm=stop)
#     time.sleep(10)
robot.stage3(stop_cm=10)

# robot.stage4(stop_cm=20, stop_angle=30)
# robot.stage4(speed=0.1, stop_cm=10, stop_angle=30, sign_given=False)


# while True:
#     blob = robot.get_snap(8, show_one_blob=False, debug_mode=True, pix_thresh=2000)
#     print(blob)

