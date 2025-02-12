import time, sensor
from camera import *
from servos import *
from machine import LED
def get_snap(curr_color):
    blobs = [b for b in camera.get_blobs()[0] if b.code() == 2**curr_color]
    return camera.get_biggest_blob(blobs)
def align_robot(cx, curr_color, count = 0):
    error = cx - (sensor.width()//2)
    if abs(error) < 50:
        move_forward(0.5)
    elif count > 5:
        move_forward(0.2)
    else:
        if error > 0:
            servo.set_speed(-0.1, 0.1)
        else:
            servo.set_speed(0.1, -0.1)
        time.sleep(0.05)
        servo.set_speed(0, 0)
        for i in range(3):
            lb = get_snap(curr_color)
            if lb:
                align_robot(lb.cx(), curr_color, count = count + 1)
                break
            move_forward(0.3)
            servo.set_speed(0, 0)
def move_forward(dur):
    servo.set_differential_drive(0.1, 0)
    time.sleep(dur)

if __name__ == "__main__":
    led = LED("LED_BLUE")
    led.on()
    servo = Servo()
    servo.soft_reset()
    thresholds = [
    (34, 46, -28, -15, 2, 24), # green
    (39, 50, -4, 13, -61, -38), #blue
    (75, 89, -23, 4, 20, 68), # new yellow
    (43, 49, 40, 59, 13, 41), # red
    (52, 61, 23, 39, 19, 50), # orange
    (28, 36, 8, 25, -16, 2), # purple
    (60, 77, -40, -18, 4, 53) # light green
    ]
    camera = Cam(thresholds, gain = 10)
    found = False
    search, curr_color = 0, 0
    while curr_color < len(thresholds):
        lb = get_snap(curr_color)
        if lb:
            found = True
            search = 0
            align_robot(lb.cx(), curr_color)
        elif found:
            found = False
            curr_color += 1
        else:
            while not lb:
                search += 1
                servo.set_speed(-0.1, 0.1)
                time.sleep(0.1)
                if search > 30:
                    move_forward(search//30*0.05)
                servo.set_speed(0, 0)
                lb = get_snap(curr_color)
    servo.set_speed(0, 0)
