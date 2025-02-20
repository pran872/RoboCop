'''
If thresholds are set properly, this runs through the whole 7 circles.
'''
from servos import *
from camera import *
import sensor
from machine import LED
import time


def get_snap(curr_color):
    all_blobs, img = camera.get_blobs()
    # if all_blobs:
    blobs_colors = camera.get_blob_colours(all_blobs)
    blobs = []
    for i in range(len(blobs_colors)):
        if blobs_colors[i] == 2**(curr_color):
            blobs.append(all_blobs[i])

    largest_blob = camera.get_biggest_blob(blobs)

    # print(all_blobs)
    # print(blobs_colors)
    # print(largest_blob)
    # print()
    return largest_blob
    # else:
    #     return None

def align_robot(cx, curr_color, count=0):
    error = cx - CENTER_X
    print('error', error)

    if abs(error) < TOLERANCE:
        move_forward(0.5)
    elif count > 5:
        print('i was confused so i moved forward')
        move_forward(0.2)
    else:
        if error > 0:
            servo.set_speed(-0.1, 0.1)
        else:
            servo.set_speed(0.1, -0.1)

        time.sleep(0.05)
        servo.set_speed(0, 0)

        for i in range(3):
            largest_blob = get_snap(curr_color)
            if largest_blob:
                align_robot(largest_blob.cx(), curr_color, count=count+1)
                break
            move_forward(0.3)
            servo.set_speed(0, 0)

def move_forward(duration=0.7):
    print("MOVE FORWARD")
    servo.set_differential_drive(0.1, SPEED_BIAS)
    time.sleep(duration)


if __name__ == "__main__":
    led = LED("LED_BLUE")
    led.on()

    servo = Servo()
    servo.soft_reset()

    thresholds = [

        # (75, 89, -23, 4, 20, 68), # new yellow
        # (39, 50, -4, 13, -61, -38), #blue
        # (34, 46, -28, -15, 2, 24), # green
        # (43, 49, 40, 59, 13, 41), # red
        # (52, 61, 23, 39, 19, 50), # orange
        # (28, 36, 8, 25, -16, 2), # purple
        # (60, 77, -40, -18, 4, 53), # light green

        # # tuesday phd area
        (56, 62, -20, 0, -27, 0), #green
        (70, 74, -8, 3, -44, -30), #blue
        (65, 72, -10, 4, 8, 37), # yellow
        # (76, 81, 26, 40, -31, -13), #red from blue
        (69, 100, 20, 41, -26, -9), # red from yellow
        # (36, 54, 9, 26, -30, -10), #old good purple facing
        (33, 60, 10, 29, -39, 5), # purple
        (79, 87, 18, 30, -24, -9), #orange
        (56, 64, -21, -1, -9, 15) #lightgreen

    ]
    camera = Cam(thresholds, gain=10)

    CENTER_X = sensor.width() // 2
    TOLERANCE = 50
    SPEED_BIAS = 0.0
    found = False
    curr_color = 0
    searching = 0

    while curr_color < len(thresholds):
        largest_blob = get_snap(curr_color)

        if largest_blob:
            print("Found blob")
            searching = 0
            found = True
            align_robot(largest_blob.cx(), curr_color)

        elif found:
            print(f"Done finding {curr_color}. Next color: {curr_color + 1}")
            found = False
            curr_color += 1

        else:
            while not largest_blob:
                searching += 1
                print(f"Searching for color: {curr_color}")
                servo.set_speed(0.1, -0.1)  # Rotate RIGHT in place to search
                time.sleep(0.1)
                if searching > 30:
                    move_forward(searching // 30 * 0.05)
                servo.set_speed(0, 0)
                largest_blob = get_snap(curr_color)

    servo.set_speed(0, 0)
    print("Reached ultimate end.")

    servo.soft_reset()

