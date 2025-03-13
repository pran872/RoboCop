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
        self.square = {"id": 1, "pix_thresh": 2000}
        self.obstacle = {"id": 2, "pix_thresh": 60}
        self.end = {"id": 4, "pix_thresh": 1000}

        self.scan_direction = 1
        self.thresholds = thresholds



    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
        self.servo.set_differential_drive(drive, steering)


    def get_snap(self, colour_code, pix_thresh=60, closest=True, debug_mode=False, rot_angle=False):
        """
        Takes a snapshot and returns the biggest blob for the specified colour (if closest=False).

        Args:
            colour_code (int): Colour code of the blob to return.
            pix_thresh (int): Minimum size of the blob in pixels.
            closest (bool): If True, returns the closest blob (not necessarily the biggest) of the
                                specified colour
            debug_mode (bool): If True, displays rectangles around all detected blobs of all colours
        """
        angle = self.servo.pan_pos if rot_angle else 0
        blobs = [
            b for b in self.cam.get_blobs(angle=angle, pix_thresh=pix_thresh)[0]
            if b.code()==colour_code
        ]

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


    def quick_scan_for_blob(self, targets_list: list):
        dir_bias = random.choice([1, -1])

        # angles = [self.scan_direction*i for i in [30, 40, 50, 60, 70, 80, 90]]
        angles = [dir_bias*i for i in [10, 20, 30, 40, 50, 60, 70, 80, 90]]
        angles_opp = [-1*ang for ang in angles]
        angles = angles + angles_opp

        for angle in angles:
            self.servo.set_angle(angle)

            for target_info in targets_list:
                pix_thresh = 200 if abs(angle) in [80, 90] else target_info["pix_thresh"] #wheel
                target_blob = self.get_snap(target_info["id"], pix_thresh=pix_thresh)

                if target_blob:
                    print("Scanned and found", target_blob)
                    return target_blob

        print("Quick scan failed!")
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

            target_blob = self.get_snap(target_id, pix_thresh=pix_thresh, rot_angle=True)
            lost = 0
            if not target_blob:
                target_blob = self.double_check_snap([{"id": target_id, "pix_thresh": pix_thresh}])

            while not target_blob:
                print("doesnt exist. turning")
                lost += 1
                if lost > 5:
                    return True
                if self.servo.pan_pos < 0:
                    self.servo.set_angle(self.servo.pan_pos+10) # turn pan angle right
                else:
                    self.servo.set_angle(self.servo.pan_pos-10) # turn pan angle left

                target_blob = self.get_snap(target_id, pix_thresh=pix_thresh, rot_angle=True)

            pan_angle = self.track_blob(target_blob, pan=True)
            time.sleep(0.1)


    def body_search(self):
        self.servo.set_angle(0)
        turn_bias = random.choice([-1, 1])
        turn_bias = [turn_bias, turn_bias*-1]

        for _ in range(12):
            self.servo.set_speed(0.1*turn_bias[0], 0.1*turn_bias[1])
            time.sleep(0.1)
            self.servo.set_speed(0, 0)

            end_blob = self.get_snap(self.end["id"], pix_thresh=self.end["pix_thresh"])
            square_blob = self.get_snap(self.square["id"], pix_thresh=self.square["pix_thresh"])
            if end_blob or square_blob:
                print("Found in first turn")
                return

        self.quick_scan_for_blob([self.end, self.square])
        self.servo.set_angle(0)

        for _ in range(22):
            self.servo.set_speed(0.1*turn_bias[1], 0.1*turn_bias[0])
            time.sleep(0.1)
            self.servo.set_speed(0, 0)

            end_blob = self.get_snap(self.end["id"], pix_thresh=self.end["pix_thresh"])
            square_blob = self.get_snap(self.square["id"], pix_thresh=self.square["pix_thresh"])
            if end_blob or square_blob:
                print("Found in second turn")
                return

        self.quick_scan_for_blob([self.end, self.square])
        self.servo.set_angle(0)

    def double_check_snap(self, targets_list):
        for i in range(5):
            for target_info in targets_list:
                print(f"Prelim search {i} for target {target_info["id"]}")
                target_blob = self.get_snap(target_info["id"], pix_thresh=target_info["pix_thresh"])
                if target_blob:
                    print("Prelim search found blob id:", target_info["id"])
                    return target_blob
        return


    def go_robot(self, speed=0.2, stop_cm=10):
        time.sleep(1)
        pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524 # curve obtained through manual points

        global_direction = 0
        reached_count = 0
        blob_no = 0
        lost_blob = False

        print("\nI am good to go. Starting now!")

        while True:
            print('blob_no', blob_no, 'global direction', global_direction)
            square_blob = self.get_snap(self.square["id"], pix_thresh=self.square["pix_thresh"])
            end_blob = self.get_snap(self.end["id"], pix_thresh=self.end["pix_thresh"])

            if end_blob:
                print("\n I found the end blob! Yahoooooo")
                self.track_and_steer(end_blob, speed)
                reached_count += 1

            elif reached_count > 3:
                print("Im done foreva")
                break

            elif square_blob:
                print("\nI found the mid blob! Yay")
                self.drive(0,0)
                real_distance = pix_to_cm((square_blob.h()/2)+square_blob.cy())

                if real_distance < stop_cm:
                    self.drive(0, 0)
                    print("\nI have reached the stop threshold.")
                    self.mf(0.7)
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

                target_blob = self.double_check_snap([self.end, self.square])
                print(target_blob)

                if not target_blob:
                    print("not. so searching")
                    target_blob = self.quick_scan_for_blob([self.end, self.square])

                    if target_blob:
                        global_direction = self.pan_to_global(self.servo.pan_pos, global_direction)
                        pix_thresh = self.square["pix_thresh"] if target_blob.code() == self.square["id"] else self.end["pix_thresh"]
                        lost_blob = self.align_body_and_eyes(target_blob.code(), pix_thresh=pix_thresh)

                    else: # no blob even after quick scan then do body search
                        print("doing body_search")
                        self.body_search()

                    if lost_blob: # lost in align body and eyes then do body search
                        print("doing body_search part 2")
                        self.body_search()
                        lost_blob = False


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

        # Pranathi's living room table - 15 gain
        (41, 70, -1, 18, -50, -27), # blue
        (0, 0, 0, 0, 0, 0), # obstacle
        (65, 75, -31, -11, -15, 11), # green

        # #RSM 301C - gain 20
        # (45, 65, -22, -2, -27, 4), #blue
        # (0, 0, 0, 0, 0, 0), # obstacle
        # (76, 91, -66, -42, 30, 66) #green
    ]


    robot = Robot(thresholds, gain = 15, p_steer=1.1, d_steer=0.005, p=0.8, i=0, d=0.005, imax=0.0)

    try:
        robot.go_robot(speed=0.1)
    except Exception as e:
        print(e)
        robot.servo.soft_reset()
