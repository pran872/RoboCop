from servos import *
from camera import *
from machine import LED

led = LED("LED_BLUE")
led.on()

servo = Servo()
servo.soft_reset()

thresholds = [
    (34, 46, -28, -15, 2, 24), # green
    (39, 50, -4, 13, -61, -38), #blue
    (75, 89, -23, 4, 20, 68), # new yellow
    (43, 49, 40, 59, 13, 41) # red
]
camera = Cam(thresholds)

# Test your assignment code here. Think about how you might want to adjust the steering based on the position of
# the colour targets in the image. What should the bot do if the next colour target is outside the field of view
# of the camera? What would your bot do if the colour target is lost for a single frame? Some helpful functions are:
# camera.get_blobs_bottom()
# camera.find_blobs()
# servos.set_differential_drive()

servo.soft_reset()
