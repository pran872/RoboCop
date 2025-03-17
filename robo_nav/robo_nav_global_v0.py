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
        self.end = {"id": 2, "pix_thresh": 1000}
        self.left = {"id": 4, "pix_thresh": 5000}
        self.right = {"id": 8, "pix_thresh": 5000}
        self.obstacle = {"id": 16, "pix_thresh": 10000}

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


    def get_snap(self, colour_code, pix_thresh=60, closest=True, debug_mode=False, rot_angle=False, width_thresh=False):
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

        if blobs and width_thresh:
            blobs = [blob for blob in blobs if blob.w()>width_thresh]

        if debug_mode:
            img = sensor.snapshot()
            for blob in img.find_blobs(self.thresholds, pixels_threshold=pix_thresh):
                if width_thresh:
                    if blob.code() == colour_code and blob.w() > width_thresh:
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
            # print("Moving pan angle to", pan_angle)
            # Move pan servo to track block
            self.servo.set_angle(pan_angle)

        return pan_angle


    def quick_scan_for_blob(self, targets_list: list, enforce_random=False, bias=False):
        if enforce_random:
            print("Enforced random scanning bias")
            dir_bias = random.choice([1, -1])
        elif bias is not None:
            dir_bias = bias
        else:
            if self.global_dir < 135 and self.global_dir > 45:
                print("Left scanning bias")
                dir_bias = -1
            elif self.global_dir > 225 and self.global_dir < 315:
                print("Right scanning bias")
                dir_bias = 1
            else: # at global dirs 135-225, 315-360, 0-45
                print("Random scanning bias")
                dir_bias = random.choice([1, -1])

        # angles = [self.scan_direction*i for i in [30, 40, 50, 60, 70, 80, 90]]
        angles = [dir_bias*i for i in [10, 20, 30, 40, 50, 60, 70, 80, 90]]
        # angles = [dir_bias*i for i in [10, 20, 30, 40, 50, 60, 70]]
        angles_opp = [-1*ang for ang in angles]
        angles = angles + angles_opp

        for angle in angles:
            self.servo.set_angle(angle)

            for target_info in targets_list:
                pix_thresh = 200 if abs(angle) in [80, 90] else target_info["pix_thresh"] #wheel
                target_blob = self.get_snap(target_info["id"], pix_thresh=pix_thresh)

                if target_blob:
                    target_blob = self.double_check_snap([target_info])
                    if target_blob:
                        print("Scanned and found", target_blob)
                        return target_blob, angle
                    else:
                        continue

        print("Quick scan failed!")
        self.servo.set_angle(0)
        return None, None

    def pan_to_global(self, pan_angle):
        if pan_angle > 0:
            print('more than', pan_angle, self.global_dir)
            result = pan_angle + self.global_dir
        elif pan_angle < 0:
            print('less than', pan_angle, self.global_dir)
            result = pan_angle + self.global_dir + 360
        else:
            result = self.global_dir

        self.global_dir = result % 360


    def mf(self, duration=0.5):
        self.drive(0.1, 0.2)
        time.sleep(duration)
        self.drive(0, 0)

    def mb(self, duration=0.5):
        print("move backwards")
        self.servo.set_speed(-0.1, -0.1)
        time.sleep(duration)
        self.drive(0, 0)

    def track_and_steer(self, target_blob, speed=0.1):
        pan_angle = self.track_blob(target_blob)
        # self.drive(speed, max(-0.1, min(pan_angle*-1/60, 0.4)))
        steering = max(min(pan_angle*-1/50, 0.2), -0.2)
        self.drive(speed, steering)
        print("In track and steer. Steering by:", steering)
        # self.mf(1)
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
                target_blob = self.double_check_snap([{"id": target_id, "pix_thresh": pix_thresh}], count=2)

            while not target_blob:
                print("doesnt exist. turning")
                lost += 1
                if lost > 3:
                    return True
                if self.servo.pan_pos < 0:
                    self.servo.set_angle(self.servo.pan_pos+10) # turn pan angle right
                else:
                    self.servo.set_angle(self.servo.pan_pos-10) # turn pan angle left

                target_blob = self.get_snap(target_id, pix_thresh=pix_thresh, rot_angle=True)

            pan_angle = self.track_blob(target_blob, pan=True)
            time.sleep(0.1)


    def double_check_snap(self, targets_list, count=2):
        for i in range(count):
            for target_info in targets_list:
                # print(f"Prelim search {i} for target {target_info["id"]}")
                target_blob = self.get_snap(target_info["id"], pix_thresh=target_info["pix_thresh"])
                if target_blob:
                    # print("Prelim search found blob id:", target_info["id"])
                    return target_blob
        return


    def turn_ish(self, direction, target_blobs_info):
        print("in turnish")
        turn_bias = [1, -1] if direction == "right" else [-1, 1]
        for tb_info in target_blobs_info:
            target_blob = self.get_snap(tb_info["id"], pix_thresh=tb_info["pix_thresh"])
            if target_blob:
                return

        count = 0
        while count < 10:
            count += 1
            self.servo.set_speed(0.1*turn_bias[0], 0.1*turn_bias[1])
            time.sleep(0.1)
            self.servo.set_speed(0, 0)
            time.sleep(0.05)
            # for tb_info in target_blobs_info:
            #     target_blob = self.get_snap(tb_info["id"], pix_thresh=tb_info["pix_thresh"])
            #     if target_blob:
            #         print("Spotted target. Ending turning")
            #         return


    def obstacle_turn(self):
        obstacle_blob = self.get_snap(self.obstacle["id"], pix_thresh=self.obstacle["pix_thresh"])
        if obstacle_blob:

            if self.global_dir > 45 and self.global_dir < 135: # turn left
                print("left bias")
                turn_bias = [-1, 1]
            elif self.global_dir > 225 and self.global_dir < 315: # turn right
                print("right bias")
                turn_bias = [1, -1]
            else:
                print("random bias")
                turn_bias = random.choice([1, -1])
                turn_bias = [turn_bias, turn_bias*-1]

            def turn_scan(turn_bias):
                while abs(self.servo.pan_pos) < 90:
                    obstacle_blob = self.get_snap(self.obstacle["id"], pix_thresh=self.obstacle["pix_thresh"])
                    if not obstacle_blob: print("Lost obstacle")

                    self.track_blob(obstacle_blob)
                    self.servo.set_speed(0.1*turn_bias[0], 0.1*turn_bias[1])
                    time.sleep(0.1)
                    self.servo.set_speed(0, 0)
                return self.quick_scan_for_blob([self.end, self.square], enforce_random=True)

            target_blob = turn_scan(turn_bias)
            if not target_blob:
                target_blob = turn_scan([turn_bias[0]*-1, turn_bias[1]*-1])

            return target_blob

        else:
            print("Didn't see obstacle")
            obstacle_blob = self.double_check_snap([self.obstacle], count=5)
            return self.obstacle_turn() if obstacle_blob else None

    def go_robot(self, speed=0.2, stop_cm=8, global_direction = 0, bias = None):
        time.sleep(1)
        pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524 # curve obtained through manual points

        reached_count = 0
        blob_no = 0
        lost_blob = False
        self.global_dir = global_direction
        # prev_actions = [None, None, None, None, None]

        print("\nI am good to go. Starting now!")

        while True:
            # prev_actions.pop(0)

            print('blob_no', blob_no, 'global direction', self.global_dir)
            square_blob = self.get_snap(self.square["id"], pix_thresh=self.square["pix_thresh"])
            # square_blob = self.get_snap(self.obstacle["id"], pix_thresh=self.obstacle["pix_thresh"])

            end_blob = self.get_snap(self.end["id"], pix_thresh=self.end["pix_thresh"])
            left_blob = self.get_snap(self.left["id"], pix_thresh=self.left["pix_thresh"], width_thresh=400)
            right_blob = self.get_snap(self.right["id"], pix_thresh=self.right["pix_thresh"], width_thresh=400)

            if end_blob:
                # prev_actions.append("end")
                print("\nI found the end blob! Yahoooooo")
                # break

                pan_angle = self.track_and_steer(end_blob, speed)
                self.pan_to_global(pan_angle)
                reached_count += 1
                print('reached_count', reached_count)

            elif reached_count > 2:
                print("Im done foreva")
                break

            elif square_blob:
                # prev_actions.append("square")
                # print("\nI found the mid blob! Yay")
                # self.drive(0,0)
                real_distance = pix_to_cm((square_blob.h()/2)+square_blob.cy())
                print("Real distance not stop:", real_distance)

                if real_distance < stop_cm:
                    print("Real distance:", real_distance)
                    # self.drive(0, 0)
                    # time.sleep(2)
                    print("\nI have reached the stop threshold.")
                    self.mf(0.7)
                    if self.global_dir < 50 or self.global_dir > 305:
                        print("Incrementing blob_no. Global direction:", self.global_dir)
                        blob_no += 1
                    else:
                        print(f"Not incrementing blob_no as global direction is {self.global_dir}")
                else:
                    # print('oi')
                    pan_angle = self.track_and_steer(square_blob, speed)
                    self.pan_to_global(pan_angle)

            elif left_blob: #and "left" not in prev_actions:
                # prev_actions.append("left")
                # self.double_check_snap([self.left], count=5)
                print("\nI hit the left side of the map. Turning right")
                self.mb(0.2)
                self.turn_ish(direction="right", target_blobs_info=[self.end, self.square])
                self.global_dir = 0
                # break

            elif right_blob:
                print("\nI hit the right side of the map. Turning left")
                self.mb(0.2)
                self.turn_ish(direction="left", target_blobs_info=[self.end, self.square])
                self.global_dir = 0


            else: # Cannot find any blob
                print("\nCannot find any blob")
                self.drive(0, 0)
                # break
                self.mb(0.7)
                self.drive(0, 0)

                target_blob = self.double_check_snap([self.end, self.left, self.right, self.square], count=2)

                if not target_blob:
                    target_blob, scan_angle = self.quick_scan_for_blob([self.end, self.square], bias=bias)

                    if target_blob:
                        print("scan_angle", scan_angle)
                        pix_thresh = self.square["pix_thresh"] if target_blob.code() == self.square["id"] else self.end["pix_thresh"]
                        lost_blob = self.align_body_and_eyes(target_blob.code(), pix_thresh=pix_thresh)

                        if not lost_blob:
                            self.pan_to_global(scan_angle)

                    else: # no blob even after quick scan then do body search
                        print("Doing obstacle turn")
                        self.obstacle_turn()
                else:
                    print(target_blob)


        self.drive(0, 0)
        self.servo.soft_reset()



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
        # (41, 70, -1, 18, -50, -27), # blue
        # (41, 74, -1, 18, -50, -30), # blue 2
        # (65, 75, -31, -11, -15, 11), # green
        # (0, 0, 0, 0, 0, 0), # left
        # # (59, 78, 9, 30, -28, -2), # right pink 1
        # (59, 78, 16, 30, -28, -2), # right pink 2
        # (48, 65, 19, 40, -34, -4), # obstacle,

        # Pranathi house 2 - 15 gain
        (12, 38, -4, 18, -41, -7), # blue
        (49, 73, -57, -20, 15, 56), # green
        (21, 52, 13, 36, -20, 7), #left pink
        (54, 76, -14, 1, 17, 60), # right yellow
        (19, 32, 30, 48, 13, 42), # obstacle red

        #RSM 301C - gain 20
        # (45, 65, -22, -2, -27, 4), #blue
        # (0, 0, 0, 0, 0, 0), # obstacle
        # (76, 91, -66, -42, 30, 66) #green

        # self.square = {"id": 1, "pix_thresh": 2000}
        # self.end = {"id": 2, "pix_thresh": 1000}
        # self.left = {"id": 4, "pix_thresh": 5000}
        # self.right = {"id": 8, "pix_thresh": 5000}
        # self.obstacle = {"id": 16, "pix_thresh": 10000}
    ]


    robot = Robot(thresholds, gain = 15, p_steer=1.1, d_steer=0.005, p=1, i=0, d=0.005, imax=0.0)

    # while True:
    #     robot.get_snap(robot.left["id"], pix_thresh=robot.left["pix_thresh"], width_thresh=400, debug_mode=True)
    # try:
    robot.go_robot(speed=0.1, global_direction=45, stop_cm=12)
    # except Exception as e:
    #     print(e)
    # robot.servo.soft_reset()
