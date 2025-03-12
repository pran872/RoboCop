from servos import *
from camera import *
from pid_control import PID
import time
from machine import LED
import random

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain, p_steer=1.1, d_steer=0.005, p=0.18, i=0, d=0, imax=0.01):
        """
        Initializes the Robot object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds.
            gain (float): Camera gain.
            p_steer (float): Proportional gain for steering the robot PID.
            d_steer (float): Derivative gain for steering the robot PID.
            p (float): Proportional gain for the pan angle PID.
            i (float): Integral gain for the pan angle PID.
            d (float): Derivative gain for the pan angle PID.
            imax (float): Maximum Integral error for the pan angle PID.
        """
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)
        self.PID = PID(p, i, d, imax)
        self.PID_steering = PID(p_steer, 0, d_steer, imax=0.0)

        # Blob IDs
        self.square_id = 1
        self.obstacle_id = 2
        self.end_id = 4

        self.scan_direction = -1
        self.thresholds = thresholds


    def debug_tracking(self, colour_code=0):
        # blob = self.get_snap(colour_code)
        # pan_angle = self.track_blob(blob)

        # self.servo.set_speed(0.1,-0.1) # turns right
        # time.sleep(0.04)

        # self.servo.set_speed(0, 0)

        # print(pan_angle)

        for i in [-10, -20, -30, -40, -50, -60]:
            print(i)
            self.servo.set_angle(i)
            self.get_snap(0, debug_mode=True)
            time.sleep(2)


    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
        self.servo.set_differential_drive(drive, steering)


    def get_snap(self, colour_code, pix_thresh=60, closest=False, debug_mode=False, rot_angle=False):
        """
        Takes a snapshot and returns the biggest blob for the specified colour (if closest=False).

        Args:
            colour_code (int): Colour code of the blob to return.
            pix_thresh (int): Minimum size of the blob in pixels.
            closest (bool): If True, returns the closest blob (not necessarily the biggest) of the
                                specified colour
            debug_mode (bool): If True, displays rectangles around all detected blobs of all colours
        """
        if rot_angle:
            blobs=[b for b in self.cam.get_blobs(angle=self.servo.pan_pos, pix_thresh=pix_thresh)[0]
                    if b.code()==colour_code]
        else:
            blobs=[b for b in self.cam.get_blobs(pix_thresh=pix_thresh)[0] if b.code()==colour_code]

        if debug_mode:
            img = sensor.snapshot()
            for blob in img.find_blobs(self.thresholds, pixels_threshold=pix_thresh):
                img.draw_rectangle(blob.rect())
                img.draw_string(blob.cx(), blob.cy(), str(int(blob.code())), scale=2)

        if blobs and closest:
            closest_blob = sorted(blobs, key=lambda b: b.cy())
            return closest_blob[-1]

        return self.cam.get_biggest_blob(blobs)


    def track_blob(self, blob, pan=True) -> None:
        """
        Adjust the camera pan angle to track a specified blob based on its ID.

        Args:
            blob: The blob object to be tracked
        """
        # Error between camera angle and target in pixels
        pixel_error = blob.cx() - self.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/sensor.width()*self.cam.h_fov)

        pid_error = self.PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        if pan:
            print("Moving pan angle to", pan_angle)
            # Move pan servo to track block
            self.servo.set_angle(pan_angle)

        return pan_angle


    def quick_scan_for_blob(self, code_ids: list, pix_thresholds = [60]):
        print("in quick scan")

        angles = [self.scan_direction*i for i in [30, 40, 50, 60, 70, 80, 90]]
        angles_opp = [-1*ang for ang in angles]
        angles = angles + angles_opp
        print("angless", angles)

        for angle in angles:
            self.servo.set_angle(angle)

            for pix_thresh, code_id in zip(pix_thresholds, code_ids):
                if abs(angle) in [80, 90]:
                    pix_thresh = 200

                target_blob = self.get_snap(code_id, pix_thresh=pix_thresh)

                if target_blob:
                    final_pan_angle = self.servo.pan_pos + (self.scan_direction * 6)

                    target_blob = self.get_snap(code_id, pix_thresh=pix_thresh)
                    print("pan_angleee", angle, final_pan_angle)
                    print("Scanned and found", target_blob)
                    if target_blob is None:
                        print("Changing final angle lost blob", angle)
                        self.set_angle(angle)
                        target_blob = self.get_snap(code_id, pix_thresh=pix_thresh)
                        print("target_blob", target_blob)

                    return target_blob

        return None



    def scan_for_blob(self, threshold_ids: list, step = 2, limit = 20, pix_thresholds = [60]) -> None:

        if isinstance(threshold_ids, int):
            threshold_ids = [threshold_ids]

        if isinstance(pix_thresholds, int):
            pix_thresholds = [pix_thresholds]

        scan_count = 0
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            # blobs, _ = self.cam.get_blobs(self.servo.pan_pos)
            for pix_thresh, threshold in zip(pix_thresholds, threshold_ids):
                blobs, _ = self.cam.get_blobs(pix_thresh=pix_thresh)
                found_idx = self.cam.find_blob(blobs, threshold)

                if found_idx is not None:
                    final_pan_angle = self.servo.pan_pos + (self.scan_direction * 6)
                    print("pan_angle", new_pan_angle, final_pan_angle)
                    print("Scanned and found", found_idx)
                    while found_idx is None:
                        print('in loop')
                        blobs, _ = self.cam.get_blobs(pix_thresh=pix_thresh)
                        found_idx = self.cam.find_blob(blobs, threshold)
                    print(found_idx)
                    return blobs[found_idx]


            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                scan_count += 1
                self.scan_direction *= -1

                if scan_count > 1:
                    print("Scanned and not found")
                    return None


    def pan_to_global(self, pan_angle, global_direction):
        if pan_angle > 0:
            print('more than', pan_angle, global_direction)
            result = pan_angle + global_direction
        elif pan_angle < 0:
            print('less than', pan_angle, global_direction)
            result = pan_angle + global_direction + 360
        else:
            result = global_direction
        return result % 360


    def mf(self, duration=0.5):
        self.drive(0.1, 0)
        time.sleep(duration)
        self.drive(0, 0)

    def mb(self, duration=0.5):
        self.servo.set_speed(-0.1, -0.1)
        time.sleep(duration)
        self.drive(0, 0)

    def track_and_steer(self, target_blob, speed=0.1):
        pan_angle = self.track_blob(target_blob)
        self.drive(speed, pan_angle*-1/60)
        print("In track and steer. Steering by:", pan_angle*-1/60)
        return pan_angle


    def align_body_and_eyes(self, target_id, pix_thresh=60):
        pan_angle = self.servo.pan_pos
        while True:
            if abs(pan_angle) < 15:
                print('\nAligned. Breaking')
                break
            else:
                if pan_angle < 0: #-ve stop angle
                    self.servo.set_speed(-0.1, 0.1) # turn body left
                    time.sleep(0.07)
                else: #+ve stop angle
                    self.servo.set_speed(0.1, -0.1) # turn body right
                    time.sleep(0.05)

                self.servo.set_speed(0, 0)
                # time.sleep(0.05)

            target_blob = self.get_snap(target_id, closest=True, pix_thresh=pix_thresh, rot_angle=True)
            lost = 0
            while not target_blob:
                lost += 1
                if lost > 5:
                    return
                print("doesnt exist. turning")
                if self.servo.pan_pos < -10:
                    self.servo.set_angle(self.servo.pan_pos+5) # turn pan angle right
                else:
                    self.servo.set_angle(self.servo.pan_pos-5) # turn pan angle left

                target_blob = self.get_snap(target_id, closest=True, pix_thresh=pix_thresh, rot_angle=True)

            pan_angle = self.track_blob(target_blob, pan=True)
            time.sleep(0.1)

        self.drive(0, 0)


    def body_search(self):
        # turn_bias = random.choice([-1, 1])
        # turn_bias = [turn_bias, turn_bias*-1]
        if self.scan_direction == 1:
            turn_bias = [1, -1]
        else:
            turn_bias = [-1, 1]

        for i in range(12):
            self.servo.set_speed(0.1*turn_bias[0], 0.1*turn_bias[1])
            time.sleep(0.1)
            self.servo.set_speed(0, 0)

            end_blob = self.get_snap(self.end_id, pix_thresh=1000)
            square_blob = self.get_snap(self.square_id, pix_thresh=2000)
            if end_blob or square_blob:
                print("Found in first turn")
                return

        for i in range(22):
            self.servo.set_speed(0.1*turn_bias[1], 0.1*turn_bias[0])
            time.sleep(0.1)
            self.servo.set_speed(0, 0)

            end_blob = self.get_snap(self.end_id, pix_thresh=60)
            square_blob = self.get_snap(self.square_id, pix_thresh=2000)
            if end_blob or square_blob:
                print("Found in second turn")
                return



    def go_robot(self, speed=0.2, stop_cm=10):

        time.sleep(1)
        global_direction = 0
        reached_end = False
        reached_count = 0
        pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524 # curve obtained through manual points

        print("\nI am good to go. Starting now!")

        blob_no = 0


        while True:
            print('blob_no', blob_no, 'global direction', global_direction)
            square_blob = self.get_snap(self.square_id, closest=True, pix_thresh=2000)
            end_blob = self.get_snap(self.end_id, closest=True, pix_thresh=1000)

            if end_blob:
                print("\n I found the end blob! Yahoooooo")
                self.track_and_steer(end_blob, speed)
                reached_count += 1
                if reached_count > 3:
                    reached_end = True

            elif reached_end:
                print("Im done foreva")
                break

            elif square_blob:
                print("\nI found the mid blob! Yay")
                self.drive(0,0)
                real_distance = pix_to_cm((square_blob.h()/2)+square_blob.cy())


                if real_distance < stop_cm:
                    self.drive(0, 0)
                    print("\nI have reached the stop threshold.")
                    self.mf(0.5)
                    if global_direction < 45 or global_direction > 315:
                        print("Incrementing blob_no:", global_direction)
                        blob_no += 1
                    else:
                        print(f"Not incrementing blob_no as global direction is {global_direction}")
                else:
                    self.track_and_steer(square_blob, speed)


            else: # Cannot find any blob
                print("\nCannot find mid blob")
                self.drive(0, 0)
                if blob_no < 4:
                    print("Blob less than 4 so moving forward")
                    self.mf(0.7)

                for i in range(5):
                    print("Prelim search", i)
                    square_blob = self.get_snap(self.square_id, closest=True, pix_thresh=2000)
                    if square_blob:
                        print("Prelim search found blob", i)
                        break
                else:
                    # target_blob = self.scan_for_blob([2, 0], limit=90, step=10, pix_thresholds=[60, 2000])
                    target_blob = self.quick_scan_for_blob([4, 1], pix_thresholds=[1000, 2000])
                    # target_blob = None

                    if target_blob:
                        print("Target blob code:", target_blob.code())

                        if target_blob.code() == 1: # square code
                            global_direction = self.pan_to_global(self.servo.pan_pos, global_direction)
                            pix_thresh = 2000
                        else:
                            pix_thresh=60
                        self.align_body_and_eyes(target_blob.code(), pix_thresh=pix_thresh)
                    else: # no blob even after searching

                        self.servo.set_angle(0)
                        print("doing body_search")
                        self.body_search()

        self.drive(0, 0)
        # self.servo.soft_reset()



if __name__ == "__main__":
    led = LED("LED_BLUE")
    led.on()

    thresholds = [
        # RSM 301 D - gain 10
        # (17, 25, -9, 7, -26, -12), # blue square
        # (18, 36, 28, 47, 9, 38), # obstacle

        # Aronnys's bedroom floor - gain 20
        # (7, 15, -14, 10, -24, -6), # blue
        # (0, 0, 0, 0, 0, 0),
        # (31, 62, -54, -20, 8, 50), # green

        # Pranathi's living room table - gain 10
        (31, 42, -9, 12, -40, -22), # blue 1
        (0, 0, 0, 0, 0, 0), # obstacle
        (37, 52, -32, -13, -5, 26)
    ]


    robot = Robot(thresholds, gain = 10, p_steer=1.1, d_steer=0.005, p=0.8, i=0, d=0.005, imax=0.0)

    try:
        robot.go_robot(speed=0.1)
    except:
        robot.servo.soft_reset()
