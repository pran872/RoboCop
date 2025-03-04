from robot import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [
    #Pranathi's living room table - gain 10 with lamp
    # (50, 63, 20, 33, -15, 7), # red
    # (63, 69, 20, 38, -16, 5), # red 2 - phone as well
    # (70, 76, 27, 41, -18, 2), # red 3
    # (56, 77, 14, 48, -15, 12), # red 4
    # (59, 65, 23, 39, -15, 12), # red 5
    # (65, 71, 23, 39, -15, 12), # red 6
    # (56, 68, 24, 41, -15, 12), # red 7
    # # (37, 49, 12, 21, -34, -19), # purple
    # (44, 56, 15, 29, -32, -8), # purple 2
    # # (57, 69, -17, -4, -26, -11) # light blue
    # # (64, 80, -15, -4, -27, -6) # light blue 2
    # (73, 77, -8, 2, -30, -16), # light blue 3
    # # (44, 63, -9, 10, -15, 12), # green paper obstacle
    #  (7, 18, -11, 9, -15, 7) # black obstacle

    # Pranathi's bedroom floor - gain 10 - lamp + phone
    # (34, 43, 14, 38, 0, 32), # red 1
    (39, 56, 16, 38, -6, 20), # red 2
    # (47, 57, -23, -7, -20, -3), # light blue 1 LEFT
    # (40, 56, -14, 0, -26, -13), # light blue 2 LEFT
    (54, 71, -14, 1, -25, -15), # light blue 3 LEFT
    # (8, 23, -2, 18, -18, 1), # purple 1 RIGHT
    (13, 28, 8, 25, -25, -7), # purple 2 RIGHT
    # (9, 42, -11, 5, 13, 41) # obstacle yellow 1
    (16, 47, -5, 12, 14, 37), # obstacle yellow 2



    #Rachael's Skempton thresholds
    # (41, 46, 27, 42, 7, 35), # red
    # (42, 54, -20, -1, -29, -6), # bluex
    # (19, 29, -8, 31, -22, 13), # purple
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
# robot.stage3(stop_cm=20)

robot.stage4(stop_cm=20, stop_angle=30)


# while True:
    # blob = robot.get_snap(8, show_one_blob=False, debug_mode=True, pix_thresh=2000)
#     print(blob)

