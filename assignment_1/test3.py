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
        time.sleep(0.7)
    else:
        if error > 0:
            servo.set_speed(-0.1, 0.1)
        else:
            servo.set_speed(0.1, -0.1)

        time.sleep(0.05)
        servo.set_speed(0, 0)
        # time.sleep(0.1)

        largest_blob = get_snap(curr_color)
        if largest_blob:
            align_robot(largest_blob.cx(), largest_blob.cy(), curr_color)
        else:
            print('in align return')
            for i in range(3):
                time.sleep(0.5)
                largest_blob = get_snap(curr_color)
                if largest_blob:
                    print("align_found")
                    break
                move_forward()
                time.sleep(0.1)
                servo.set_speed(0, 0)
            else:
                print('returning align')
                # exit()
                return

            align_robot(largest_blob.cx(), largest_blob.cy(), curr_color)
        # if largest_blob:
        #     return align_robot(largest_blob.cx(), largest_blob.cy(), curr_color)
        # else:
        #     print("need to retry")

        #     return True


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
    # (58, 82, -31, -5, -7, 18), # green
    (54, 65, -12, 2, -48, -27), # blue
    (88, 95, -12, 0, 14, 60), # yellow
    ]
    camera = Cam(thresholds, gain=10)

    CENTER_X = sensor.width() // 2
    TOLERANCE = 50
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

            retry = align_robot(largest_blob.cx(), largest_blob.cy(), curr_color)
            # if retry:
            #     exit()
            #     found = False
            #     print('made found False')
            #     time.sleep(1)


        while not largest_blob:
            if found:
                print(f"Done finding {curr_color}. Next")
                curr_color += 1
                if curr_color > TOTAL_COLOURS:
                    move_forward()
                    time.sleep(0.8)
                    servo.set_speed(0, 0)
                    ultimate_end = True
                else:
                    found = False
                break


            print(f"Searching for color: {curr_color}")
            servo.set_speed(-0.1, 0.1)  # Rotate LEFT in place to search
            time.sleep(0.1)
            move_forward()
            time.sleep(0.05)
            servo.set_speed(0, 0)
            largest_blob = get_snap(curr_color)

        if ultimate_end:
            print("Reached ultimate end. Exiting")
            break

    servo.soft_reset()
