from servos import *
from camera import *
import sensor
from machine import LED
import time
import math

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

thresholds = [
    (13, 26, -29, -15, 8, 29) # green
    # (43, 62, -8, 15, -47, -27) #blue
]
camera = Cam(thresholds, gain=20)

CENTER_X = sensor.width() // 2
TOLERANCE = 50
SPEED_BIAS = 0.35


def get_snap():
    img = sensor.snapshot()
    img.rotation_corr(z_rotation=0)
    blobs = img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150)
    return blobs

def align_robot(cx, cy):
    error = cx - CENTER_X
    print('error', error)

    if abs(error) < TOLERANCE:
        move_forward()
    else:
        if error > 0:
            print("turn left")
            servo.set_speed(-0.1, 0.1)
        else:
            print("turn right")
            servo.set_speed(0.1, -0.1)

        print("i am")
        time.sleep(0.1)
        servo.set_speed(0, 0)
        time.sleep(0.1)

        blobs = get_snap()
        largest_blob = camera.get_biggest_blob(blobs)
        align_robot(largest_blob.cx(), largest_blob.cy())

def move_forward():

    print("Move forward")
    servo.set_differential_drive(0.1, SPEED_BIAS)
    time.sleep(1)


found = False
ultimate_end = False
while True:
    blobs = get_snap()

    if blobs:

        print('blobs', blobs)
        found = True
        largest_blob = camera.get_biggest_blob(blobs)

        print("Blob center:", largest_blob.cx(), largest_blob.cy())

        align_robot(largest_blob.cx(), largest_blob.cy())


    while not blobs:
        if found:
            print("Found before. Exiting")
            servo.set_speed(0, 0)
            ultimate_end = True
            break
        print("Searching for blobs...")
        servo.set_speed(-0.1, 0.1)  # Rotate LEFT in place to search
        time.sleep(0.1)
        servo.set_speed(0, 0)
        time.sleep(0.5)
        blobs = get_snap()

    if ultimate_end:
        break

    # servo.soft_reset()
