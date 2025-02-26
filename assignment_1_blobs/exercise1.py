from servos import *
from camera import *
from machine import LED
import sensor,time

def get_snap(col):
    blobs=[b for b in camera.get_blobs()[0] if b.code()==2**col]
    return camera.get_biggest_blob(blobs)

def align_robot(cx, col, count=0):
    error = cx-sensor.width()//2
    if abs(error)<50:
        mf(0.5)
    elif count>5:
        mf(0.2)
    else:
        if error > 0:
            servo.set_speed(-0.1,0.1)
        else:
            servo.set_speed(0.1,-0.1)
        time.sleep(0.05)
        servo.set_speed(0,0)
        for i in range(3):
            lb = get_snap(col)
            if lb:
                align_robot(lb.cx(), col, count=count+1)
                break
            mf(0.3)
            servo.set_speed(0,0)
def mf(dur):
    servo.set_differential_drive(0.1,0.05)
    time.sleep(dur)

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

thresholds = [
              (67, 76, 24, 38, -1, 29),
              (65, 81, -20, -3, -35, -20),
              (56, 70, -35, -15, -2, 29),
              (74, 80, 18, 32, 0, 24),
]
camera = Cam(thresholds,gain=20)
found = False
search, col = 0,0
while col < len(thresholds):
    lb = get_snap(col)
    if lb:
        found = True
        search = 0
        align_robot(lb.cx(),col)
    elif found:
        found = False
        col+=1
    else:
        while not lb:
            search+=1
            servo.set_speed(0.1,-0.1)
            time.sleep(0.1)
            if search>30:
                mf(search//30*0.05)
            servo.set_speed(0,0)
            lb=get_snap(col)
mf(0.2)
servo.set_speed(0,0)
servo.soft_reset()
