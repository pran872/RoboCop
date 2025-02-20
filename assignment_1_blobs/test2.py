'''
If thresholds are set properly, this runs through the whole 7 circles.
'''
from servos import *
from camera import *
import sensor
from machine import LED
import time


def get_snap(curr_color=None):
    img = sensor.snapshot()
    img.rotation_corr(z_rotation=0)
    blobs = img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150)
    if blobs and curr_color is not None:
        blobs = [blob for blob in blobs if blob.code()==2**(curr_color-1)]
        largest_blob = camera.get_biggest_blob(blobs)
        return largest_blob
    else:
        return blobs

def align_robot(cx, cy, curr_color):
    error = cx - CENTER_X
    print('error', error)

    if abs(error) < TOLERANCE:
        move_forward()
        time.sleep(0.8)
    else:
        if error > 0:
            # print("turn left")
            servo.set_speed(-0.1, 0.1)
        else:
            # print("turn right")
            servo.set_speed(0.1, -0.1)

        time.sleep(0.1)
        servo.set_speed(0, 0)
        time.sleep(0.1)

        largest_blob = get_snap(curr_color)
        if largest_blob:
            align_robot(largest_blob.cx(), largest_blob.cy(), curr_color)
        else:
            print("need to retry")

            return "retry"


def move_forward():
    # print("Move forward")
    servo.set_differential_drive(0.1, SPEED_BIAS)

# def search(direction):


if __name__ == "__main__":
    led = LED("LED_BLUE")
    led.on()

    servo = Servo()
    servo.soft_reset()

    thresholds = [
        (39, 66, -37, -10, 3, 29), # green
        (45, 74, -9, 15, -53, -28), # blue
        (82, 87, -18, -1, 10, 56), # yellow
        (47, 78, 14, 54, -2, 37), # red
        (23, 44, 6, 33, -23, 7), # purple
        (59, 69, 15, 35, 8, 44), # orange
        (62, 72, -33, -12, 9, 46), # light green

    ]
    camera = Cam(thresholds, gain=20)

    CENTER_X = sensor.width() // 2
    TOLERANCE = 100
    SPEED_BIAS = 0.35
    TOTAL_COLOURS = len(thresholds)
    found = False
    ultimate_end = False
    curr_color = 1
    while True:
        print(f"\n\n\nLooking for color: {curr_color}")
        largest_blob = get_snap(curr_color)

        if largest_blob:
            print("in largest blob")

            found = True

            out = align_robot(largest_blob.cx(), largest_blob.cy(), curr_color)
            if out == "retry":
                found = False


        while not largest_blob:
            if found:
                print(f"Done finding {curr_color}. Next")
                curr_color += 1
                if curr_color > TOTAL_COLOURS:
                    servo.set_speed(0, 0)
                    ultimate_end = True
                else:
                    found = False
                break


            print(f"Searching for color: {curr_color}")
            servo.set_speed(-0.1, 0.1)  # Rotate LEFT in place to search
            time.sleep(0.1)
            move_forward()
            time.sleep(0.1)
            servo.set_speed(0, 0)
            largest_blob = get_snap(curr_color)

        if ultimate_end:
            print("Reached ultimate end. Exiting")
            break

    # servo.soft_reset()
