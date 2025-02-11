'''
If thresholds are set properly, this runs through the whole 7 circles.
'''
# add import sensor and import time (the others are given)
from servos import *
from camera import *
import sensor
from machine import LED
import time

# get a screenshot and find the biggest blob in the screen 
# ensure indexing is correct 
def get_snap(curr_color): # just pass current colour
    all_blobs, img = camera.get_blobs() # make sure there are 2 outputs 
    blobs_colors = camera.get_blob_colours(all_blobs) # get colours for each blob
    blobs = []
    for i in range(len(blobs_colors)): # iterate through blob colours and adds blob to blob list if colour matches colour of interest
        if blobs_colors[i] == 2**(curr_color):
            blobs.append(all_blobs[i]) 

    largest_blob = camera.get_biggest_blob(blobs) # finds the biggest of those blobs
    return largest_blob

# cx is the x position of the centre of the blob
def align_robot(cx, curr_color, count=0): 
    error = cx - CENTER_X # difference between blob centroid and image centroid
    print('error', error)

    if abs(error) < TOLERANCE: 
        move_forward(0.7) 
    elif count > 5: # change this value if you think robot is going a little crazy
        move_forward(0.2)
    else: # if not within tolerance, turn left or right
        if error > 0:
            servo.set_speed(-0.1, 0.1) # left wheel goes back and right wheel goes front to turn left
        else:
            servo.set_speed(0.1, -0.1) # turn right

        time.sleep(0.05) # can vary if code is not working. time sleep is in SECONDS
        servo.set_speed(0, 0) # to stop the robot 

        for i in range(3): # can print stuff here or increase number of iterations for debugging
            largest_blob = get_snap(curr_color)
            if largest_blob: 
                align_robot(largest_blob.cx(), curr_color, count=count+1) 
                break
            move_forward(0.3) # if you don't see the blob anymore, go forward and take a snap again
            # no need to do time.sleep again here because move_forward is coded with the duration included
            servo.set_speed(0, 0) # don't add this into the move_forward function! but remember to stop

def move_forward(duration=0.7):
    print("MOVE FORWARD")
    servo.set_differential_drive(0.1, SPEED_BIAS) # can change this to set_speed if bias is always 0
    time.sleep(duration)

# speeds are always set to 0.1, time that it is moving is different
if __name__ == "__main__":
    # originally given in exercise1.py
    led = LED("LED_BLUE")
    led.on()

    servo = Servo()
    servo.soft_reset()

    thresholds = [
        (34, 46, -28, -15, 2, 24), # green
        (39, 50, -4, 13, -61, -38), #blue
        (75, 89, -23, 4, 20, 68), # new yellow
        (43, 49, 40, 59, 13, 41), # red
        (28, 36, 8, 25, -16, 2), # purple
        (52, 61, 23, 39, 19, 50), # orange
        (60, 77, -40, -18, 4, 53), # light green

    ]
    # remember to add gain argument and set same value as camera.py
    camera = Cam(thresholds, gain=10)

    CENTER_X = sensor.width() // 2 # half of image width (horizontal)
    TOLERANCE = 50 # max acceptable distance between centre of image and centre of blob in pixels
    SPEED_BIAS = 0 # NEED TO SET THIS USING SERVOS BEFORE RUNNING THIS
    found = False 
    curr_color = 0  
    searching = 0 

    while curr_color < len(thresholds): 
        largest_blob = get_snap(curr_color) # get screenshot 

        if largest_blob:
            print("Found blob")
            searching = 0 # must make 0 when the blob is found 
            found = True 
            align_robot(largest_blob.cx(), curr_color) # remember brackets after cx()

        elif found: # no largest blob but found is true (means colour has been found)
            print(f"Done finding {curr_color}. Next color: {curr_color + 1}")
            found = False
            curr_color += 1 

        else: # found is false and no blob found
            while not largest_blob:
                searching += 1
                print(f"Searching for color: {curr_color}")
                servo.set_speed(-0.1, 0.1)  # Rotate LEFT in place to search
                time.sleep(0.1)
                if searching > 30:
                    move_forward(searching // 30 * 0.05) # increase radius of search if robot has checked 360 deg
                servo.set_speed(0, 0) # must stop the robot from moving after each movement
                largest_blob = get_snap(curr_color)

    servo.set_speed(0, 0) # this stops the robot finally
    print("Reached ultimate end.")

    servo.soft_reset()

