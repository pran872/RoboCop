from servos import *
from camera import *
from pid_control import PID
import time

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
        self.mid_line_id = 1
        self.obstacle_id = 8
        self.l_line_id = 2
        self.r_line_id = 4

        self.scan_direction = 1
        self.thresholds = thresholds


    def get_snap(self, colour_code, pix_thresh=60, filter_cy=False, debug_mode=False):
        """
        Takes a snapshot and returns the biggest blob for the specified colour (if closest=False).

        Args:
            colour_code (int): Colour code of the blob to return.
            pix_thresh (int): Minimum size of the blob in pixels.
            closest (bool): If True, returns the closest blob (not necessarily the biggest) of the
                                specified colour
            filter_cy (bool): If True, only captures blobs in the top two-thirds of the image
            debug_mode (bool): If True, displays rectangles around all detected blobs of all colours
        """
        blobs=[b for b in self.cam.get_blobs(pix_thresh=pix_thresh)[0] if b.code()==colour_code]

        # only keep the blobs that are in the top 2/3s
        if filter_cy:
            blobs = [blob for blob in blobs if blob.cy()<sensor.height()*2/3]

        if debug_mode:
            img = sensor.snapshot()
            for blob in img.find_blobs(self.thresholds, pixels_threshold=pix_thresh):
                img.draw_rectangle(blob.rect())
                img.draw_string(blob.cx(), blob.cy(), str(int(blob.code())), scale=2)


        return self.cam.get_biggest_blob(blobs)


    def track_and_steer(self, target_blob, speed=0.1):
        pan_angle = self.track_blob(target_blob)
        self.drive(speed, pan_angle*-1/60)
        # print("Steering by:", pan_angle*-1/60)


    def stage1(self, speed=0.1) -> None:
        """
        Line following algorithm. Only uses the middle line.

        Args:
            speed (float): Speed to set the servos to (-1~1)
        """
        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                self.track_and_steer(mid_blob, speed)

            else: # Cannot find any blob
                print("\nCannot find mid blob")
                self.drive(0, 0)
                mid_blob = self.scan_for_blob(0, limit=30, step=2)
                if mid_blob:
                    continue
                else:
                    break

        self.servo.soft_reset()


    def stage2(self, speed=0.1) -> None:
        """
        Obstacle detection algorithm. Immediately stops when an obstacle is visible.
        Only uses the middle line.

        Args:
            speed (float): Speed to set the servos to (-1~1).
        """

        self.servo.set_angle(0)
        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000) # get biggest obstacle blob min size 2000

            if obstacle_blob:
                print("\nOh oh there's an obstacle. I'm stopping. But I'm watching you")
                self.drive(0,0)
                self.track_blob(obstacle_blob)  # change pan_angle to the obstacle
                break

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                self.track_and_steer(mid_blob, speed)

            else: # Cannot find mid blob or obstacle blob
                print("\nCannot find mid blob")
                self.drive(0, 0)
                mid_blob = self.scan_for_blob(0, limit=30, step=2)
                if mid_blob:
                    continue
                else:
                    break

        print(f"You are at {self.servo.pan_pos} pos")
        time.sleep(5)
        print("\nOkay, I will let you go")
        self.servo.soft_reset()
        return


    def stage2_continue(self, speed=0.1) -> None:
        """
        Obstacle detection algorithm. Immediately stops when an obstacle is visible. Will continue if removed
        Only uses the middle line.

        Args:
            speed (float): Speed to set the servos to (-1~1).
        """

        self.servo.set_angle(0)
        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000) # get biggest obstacle blob min size 2000

            if obstacle_blob:
                print("\nOh oh there's an obstacle. I'm stopping.")
                self.drive(0,0)

                for i in range(5):
                    print(i)
                    obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000) # get biggest obstacle blob min size 2000
                    time.sleep(1)

                if obstacle_blob:
                    print('obstacle present for 5s, breaking')
                    self.track_blob(obstacle_blob)  # keep track of obstacle
                    break

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                pan_angle = self.track_blob(mid_blob)
                self.drive(speed, pan_angle*-1/60)
                print("Found the blob. Trying again")

            elif not obstacle_blob: # Cannot find any blob

                self.drive(0, 0)
                mid_blob = self.scan_for_blob(0, limit=30, step=2)

                if mid_blob:
                    pan_angle = self.track_blob(mid_blob)
                    self.drive(speed, pan_angle*-1/60)
                    print("Found the blob. Trying again")
                    continue

                else:
                    print("\n I did it. Hooray!")
                    break

        print(f"You are at {self.servo.pan_pos} pos")
        time.sleep(5)
        print("\nOkay, I will let you go")
        self.servo.soft_reset()
        return


    def stage3(self, speed=0.1, stop_cm=10) -> None:
        """
        Obstacle distance algorithm. Follows the middle lane until the robot is a specified distance
            away from the obstacle.

        Args:
            speed (float): Speed to set the servos to (-1~1)
            stop_cm (float or int): The distance the robot should stop from the obstacle in cm (10-25cm)
        """
        stop_cm += 5
        pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524 # curve obtained through manual points

        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000) # get biggest obstacle blob min size 2000

            if obstacle_blob: # obstacle detected
                self.drive(0,0)
                real_distance = pix_to_cm((obstacle_blob.h()/2)+obstacle_blob.cy())

                print(f"\nOh oh there's an obstacle. I'm stopping at {stop_cm-5}. Real distance (cm): {real_distance-5}")

                if real_distance < stop_cm or abs(real_distance - stop_cm) < 2:
                    self.drive(0, 0)
                    self.track_blob(obstacle_blob)

                    print(f"\nI have reached the stop threshold. But I'm watching you. You're at {self.servo.pan_pos} degrees")
                    print("\nI did it hooray!")
                    break

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                self.track_and_steer(mid_blob, speed)

            else: # Cannot find mid blob or obstacle blob
                print("\nCannot find mid blob")
                self.drive(0, 0)
                mid_blob = self.scan_for_blob(0, limit=30, step=2)
                if mid_blob:
                    continue
                else:
                    break

        self.servo.set_speed(0,0)
        time.sleep(5)
        print("\n Okay, I forgive you. I will let you go now.")
        self.servo.soft_reset()
        return


    def watch_obstacle_and_turn(self, pan_angle, stop_angle):

        # obstacle_threshold = self.code_to_threshold(self.obstacle_id)
        # scan_dir = 1 if stop_angle > 0 else -1

        while True:
            if abs(pan_angle) > abs(stop_angle):
                print('\nPan angle threshold achieved.')
                break
            else:

                obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)
                while obstacle_blob is None:
                    print("can't find obstacle 1")
                    obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)
                    # obstacle_blob = self.scan_for_blob(obstacle_threshold, step=2, limit=40, pix_thresh=2000, scan_dir=scan_dir)

                if stop_angle < 0: #-ve stop angle
                    self.servo.set_speed(0.1,-0.1) # turns right
                    time.sleep(0.1)
                else: #+ve stop angle
                    self.servo.set_speed(-0.1,0.1) # turns left
                    time.sleep(0.07)

                self.servo.set_speed(0, 0)
                time.sleep(0.1)

            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)
            while obstacle_blob is None:
                print("can't find obstacle 2")
                obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)
                # obstacle_blob = self.scan_for_blob(obstacle_threshold, step=2, limit=40, pix_thresh=2000, scan_dir=scan_dir)

            pan_angle = self.track_blob(obstacle_blob, pan=True)
            time.sleep(0.1)


    def stage4(self, speed=0.1, stop_cm=10, stop_angle=20, sign_given=True) -> None:
        """
        Obstacle distance algorithm - FOLLOWS MID BLOB!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            stop_cm (float or int): The distance the robot should stop from the obstacle in cm (10-25cm)
            stop_angle (int): The angle betweent the camera and the body of the robot after encountering
                                an obstacle (camera points at the obstacle) in degrees (+-30)
            sign_given (bool): If True, the stop_angle includes the sign else, the robot infers the direction
                            to pan from the obstacle position
        """
        stop_cm += 5
        pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524  # curve 3


        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

            if obstacle_blob: # obstacle detected
                self.drive(0,0)
                real_distance = pix_to_cm((obstacle_blob.h()/2)+obstacle_blob.cy())

                print(f"\nOh oh there's an obstacle. I'm stopping at {stop_cm-5}. Real distance (cm): {real_distance-5}")

                if real_distance < stop_cm or abs(real_distance - stop_cm) < 2:
                    print(f'\nI have reached the stop threshold. Adjusting pan angle from {self.servo.pan_pos} to {stop_angle}')
                    self.drive(0, 0)

                    pan_angle = self.track_blob(obstacle_blob, pan=False)
                    if not sign_given:
                        stop_angle = -1*stop_angle if pan_angle < 0 else stop_angle

                    self.watch_obstacle_and_turn(pan_angle, stop_angle)

                    print('\nFinal real distance (cm)', real_distance, 'Final pan angle', self.servo.pan_pos)
                    print("An obstacle blocked me and I stopped - I did it hooray!")
                    break

            if mid_blob: # Found the middle blob - then move to it
                self.track_and_steer(mid_blob, speed)

            else: # Cannot find mid blob or obstacle blob
                print("\nCannot find mid blob. Looking now.")
                self.drive(0, 0)
                mid_blob = self.scan_for_blob(0, limit=30, step=2)
                if mid_blob:
                    continue
                else:
                    print("\nI stopped because I reached the end of the track - I did it hooray!")
                    break

        self.servo.set_speed(0,0)
        return

    def code_to_threshold(self, code):
        ''' Returns the threshold index given the colour code '''
        codes = [1, 2, 4, 8]
        return codes.index(code)


    def avoid_obstacle(self, target_code, speed=0.1):
        self.servo.set_angle(0)
        time.sleep(1)
        end_t = time.ticks_ms() + 3000

        while time.ticks_ms() < end_t:
            target_blob = self.get_snap(target_code)
            if target_blob:
                self.track_and_steer(target_blob, speed)
            else:
                print("\nCannot find target blob")
                self.drive(0, 0)

                target_blob = self.scan_for_blob(self.code_to_threshold(target_code), limit=30, step=2)
                if target_blob:
                    continue
                else:
                    print("I can't seem to find this target blob with code", target_code)
                    break
        self.drive(0, 0)



    def stage5(self, speed=0.1, stop_cm=10, stop_angle=30, sign_given=True) -> None:
        """
        Obstacle distance algorithm - FOLLOWS MID BLOB!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            stop_cm (float or int): The distance the robot should stop from the obstacle in cm (10-25cm)
            stop_angle (int): The angle betweent the camera and the body of the robot after encountering
                                an obstacle (camera points at the obstacle) in degrees (+-30)
            sign_given (bool): If True, the stop_angle includes the sign else, the robot infers the direction
                            to pan from the obstacle position
        """
        print("I am running stage 5")
        stop_cm += 5
        pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524  # curve 3

        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

            if obstacle_blob: # obstacle detected
                self.drive(0,0)
                real_distance = pix_to_cm((obstacle_blob.h()/2)+obstacle_blob.cy())

                print(f"\nOh oh there's an obstacle. I'm stopping at {stop_cm-5}. Real distance (cm): {real_distance-5}")

                if real_distance < stop_cm or abs(real_distance - stop_cm) < 2:
                    print(f'\nI have reached the stop threshold. Adjusting pan angle from {self.servo.pan_pos} to {stop_angle}')
                    self.drive(0, 0)

                    pan_angle = self.track_blob(obstacle_blob, pan=False)
                    if not sign_given:
                        stop_angle = -1*stop_angle if pan_angle < 0 else stop_angle

                    self.watch_obstacle_and_turn(pan_angle, stop_angle)
                    print("This determines left or right", pan_angle)
                    self.avoid_obstacle(self.l_line_id if pan_angle > 0 else self.r_line_id, speed)
                    print("Avoided the obstacle")


            if mid_blob: # Found the middle blob - now move to it
                print("\nI found the mid blob! Yay")
                self.track_and_steer(mid_blob, speed)

            else: # Cannot find mid blob or obstacle blob
                print("\nCannot find mid blob")
                self.drive(0, 0)
                mid_blob = self.scan_for_blob(0, limit=40, step=2)
                if mid_blob:
                    continue
                else:
                    break

        self.servo.set_speed(0,0)
        return


    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
        self.servo.set_differential_drive(drive, steering)


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
            # Move pan servo to track block
            self.servo.set_angle(pan_angle)

        return pan_angle


    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 20, filter_cy = False, pix_thresh=60, scan_dir=1) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        count = 0
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (scan_dir * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            # blobs, _ = self.cam.get_blobs(self.servo.pan_pos)
            blobs, _ = self.cam.get_blobs(pix_thresh=pix_thresh)

            # only keep blobs that are in the top 2/3s
            if filter_cy:
                blobs = [blob for blob in blobs if blob.cy()<sensor.height()*2/3]

            found_idx = self.cam.find_blob(blobs, threshold_idx)
            if found_idx is not None:
                print("Scanned and found")
                return blobs[found_idx]

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                count += 1
                scan_dir *= -1

                if count > 1:
                    print("Scanned and not found")
                    return None


    def debug(self, threshold_idx: int) -> None:
        """
        A debug function for the Robots vision.
        If no block ID is specified, all blocks are detected and debugged.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
        """
        while True:
            blobs, img = self.cam.get_blobs()
            print(blobs)
            if threshold_idx is not None:
                found_idx = self.cam.find_blob(blobs, threshold_idx)
            else:
                found_idx = range(len(blobs))

            if found_idx:
                for blob in [blobs[i] for i in found_idx]:
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(),blob.cy(),str(blob.code()))

                    angle_err = blob.cx() - self.cam.w_centre
                    print('\n' * 2)
                    print('Code:       ', blob.code())
                    print('X-pos:      ',blob.cx())
                    print('Pan angle:  ', self.servo.pan_pos)
                    print('Angle err:  ', angle_err)
                    print('Angle corr: ', (angle_err-self.servo.pan_pos)/self.servo.max_deg)
                    print('Block size: ', blob.pixels())

                    time.sleep(1)

    def debug_tracking(self, colour_code):
        blob = self.get_snap(colour_code)
        pan_angle = self.track_blob(blob)

        self.servo.set_speed(0.1,-0.1) # turns right
        time.sleep(0.04)

        self.servo.set_speed(0, 0)

        print(pan_angle)


    def reset(self) -> None:
        """
        Resets the servo positions to their default states and waits.
        """
        self.servo.soft_reset()


    def release(self) -> None:
        """
        Release all servos (no wait).
        """
        self.servo.release_all()
