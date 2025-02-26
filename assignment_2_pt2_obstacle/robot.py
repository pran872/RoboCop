from servos import *
from camera import *
from pid_control import PID
import time

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 25, p=0.18, i=0, d=0, imax=0.01):
        """
        Initializes the Robot object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds
            gain (float): Camera gain
            p (float): Proportional gain for the PID.
            i (float): Integral gain for the PID.
            d (float): Derivative gain for the PID.
            imax (float): Maximum Integral error for the PID.
        """
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)
        self.PID = PID(p, i, d, imax)

        # Blob IDs
        self.mid_line_id = 1
        self.obstacle_id = 8
        self.l_line_id = 2
        self.r_line_id = 4

        self.scan_direction = 1
        self.thresholds = thresholds


    def get_snap(self, col, debug_mode=True, area_thresh=60, pix_thresh=60, show_one_blob=False):
        blobs=[b for b in self.cam.get_blobs(area_thresh=area_thresh, pix_thresh=pix_thresh)[0] if b.code()==col]
        if debug_mode:
            img = sensor.snapshot()

            for blob in img.find_blobs(self.thresholds, pixels_threshold=pix_thresh, area_threshold=area_thresh):
                if show_one_blob and blob.code()!=col:
                    next
                else:
                    # print(blob)
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(), blob.cy(), str(int(blob.code())), scale=2)
        return self.cam.get_biggest_blob(blobs)


    def align_robot(self, cx, col,count=0):
        print("\nMoving to the blob")
        error = cx-sensor.width()//2
        if abs(error)<50:
            self.mf(0.5)
        elif count>5:
            self.mf(0.2)
        else:
            if error>0:
                self.servo.set_speed(-0.1,0.1)
            else:
                self.servo.set_speed(0.1,-0.1)
            time.sleep(0.05)
            self.servo.set_speed(0,0)
            for i in range(3):
                print("\nI am in the weird edge case for loop in align")
                lb=self.get_snap(col)
                if lb:
                    self.align_robot(lb.cx(),col,count=count+1)
                    break
                self.mf(0.3)
                self.servo.set_speed(0,0)


    def mf(self, dur):
        self.servo.set_differential_drive(0.1,0)
        time.sleep(dur)

    def searching(self, blob, col):
        while not blob: # Old search method moves the whole robot
            print("\nI'm looking for the blob with id:", col)
            self.servo.set_speed(0.1,-0.1)
            time.sleep(0.1)
            self.servo.set_speed(0,0)
            blob = self.get_snap(self.mid_line_id)
        return blob


    def stage1(self, speed=0.1, forward_bias=0.24) -> None:
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.set_angle(0)
        found_mid = False

        time.sleep(3)
        print("\nI am good to go. Starting now!")

        while True:
            # get biggest blob of the mid color
            mid_blob = self.get_snap(self.mid_line_id)

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                found_mid = True
                steering = round((mid_blob.cx() - self.cam.w_centre) / (self.cam.w_centre * 2), 2)
                print(steering)
                self.drive(speed, steering)
                time.sleep_ms(100)
            else: # Cannot find middle blob

                if found_mid: # But found middle blob before, then end the function - we're done
                    print("\n I did it. Hooray!")
                    self.drive(0, 0)
                    break

                else: # Look for the blob
                    mid_blob = self.searching(mid_blob, self.mid_line_id)

        self.mf(0.3)
        self.servo.set_speed(0,0)
        self.servo.soft_reset()


    def stage2(self, speed=0.1) -> None:
        """
        Obstacle detection algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        # self.servo.soft_reset()
        time.sleep(5)
        print("\nI am good to go. Starting now!")

        while True:
            # detect blobs
            mid_blob = self.get_snap(self.mid_line_id)

            # detect obstacle
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)
            print(obstacle_blob)

            if obstacle_blob: # obstacle detected
                print("\nOh oh there's an obstacle. I'm stopping but I'm looking at you")
                self.drive(0,0)
                self.drive(speed, 0)
                time.sleep(0.3) # move forward a bit after seeing the obstacle
                self.drive(0,0)

                # keep track of obstacle and if it's actually there then end
                self.track_blob(obstacle_blob)
                break

            elif mid_blob: # no obstacle but mid line
                print("\nI found the mid blob! Yay")
                steering = round((mid_blob.cx() - self.cam.w_centre) / (self.cam.w_centre * 2), 2)
                print(steering)
                self.drive(speed, steering)
                time.sleep_ms(100)
                # self.align_robot(mid_blob.cx(), self.mid_line_id)

            # else: # no obstacle and no lane
            #     mid_blob = self.searching(mid_blob, self.mid_line_id)

        self.servo.set_speed(0,0)
        time.sleep(5)
        print("\n Okay, I will let you go")
        self.servo.soft_reset()
        return


    def stage3(self, speed=0.1, stop_cm=10) -> None:
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        # self.servo.soft_reset()
        stop_cm += 3
        pix_to_cm = lambda x: 0.000105*x**2 + -0.104068*x + 25.931873

        # time.sleep(5)
        print("\nI am good to go. Starting now!")

        while True:
            # detect blobs
            mid_blob = self.get_snap(self.mid_line_id)

            # detect obstacle
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

            if obstacle_blob: # obstacle detected
                print("\nOh oh there's an obstacle. I'm stopping but I'm looking at you")
                self.drive(0,0)
                # self.drive(speed, 0)
                while True:
                    while not obstacle_blob:
                        print('i lost it')
                        obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)
                    print('obstacle - cy', obstacle_blob.cy())
                    print('obstacle - h', obstacle_blob.h())
                    print('new pix', obstacle_blob.h()-obstacle_blob.cy())
                    real_distance = pix_to_cm(obstacle_blob.h()-obstacle_blob.cy())
                    print('real distance:', real_distance)
                    if real_distance > stop_cm:
                        print('almost there - moving')
                        steering = round((obstacle_blob.cx() - self.cam.w_centre) / (self.cam.w_centre * 2), 2)
                        print('obs steering', steering)
                        self.drive(speed, steering)
                        time.sleep_ms(100)
                        # self.drive(0, 0) # Causes jerking but is more accurate!
                    else:
                        print('reached, bye bye')
                        self.drive(0, 0)
                        break
                    obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

                # self.drive(0,0)

                # keep track of obstacle and if it's actually there then end
                self.track_blob(obstacle_blob)
                break

            elif mid_blob: # no obstacle but mid line
                print("\nI found the mid blob! Yay")
                steering = round((mid_blob.cx() - self.cam.w_centre) / (self.cam.w_centre * 2), 2)
                print('steering', steering, 'mid_blobcx', mid_blob.cx())
                self.drive(speed, min(steering, 0.35))
                time.sleep_ms(100)
                # self.align_robot(mid_blob.cx(), self.mid_line_id)

            else: # no obstacle and no lane
                print("oh oh i don't see stuff")

                for i in range(5):
                    mid_blob = self.get_snap(self.mid_line_id)
                    if mid_blob:
                        break
                else:
                    print('still dont see stuff. bye bye')
                    break
            #     mid_blob = self.searching(mid_blob, self.mid_line_id)

        self.servo.set_speed(0,0)
        time.sleep(5)
        print("\n Okay, I will let you go")
        self.servo.soft_reset()
        return


    def stage4(self, speed=0.1, stop_cm=10, angle_thresh=20) -> None:
        """
        Obstacle distance + orientation algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        stop_cm += 3
        pix_to_cm = lambda x: 0.000105*x**2 + -0.104068*x + 25.931873

        # time.sleep(5)
        print("\nI am good to go. Starting now!")

        while True:
            # detect blobs
            mid_blob = self.get_snap(self.mid_line_id)

            # detect obstacle
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

            if obstacle_blob: # obstacle detected
                print("\nOh oh there's an obstacle. I'm stopping but I'm looking at you")
                self.drive(0,0)
                # self.drive(speed, 0)
                while True:

                    while not obstacle_blob:
                        print('i lost it')
                        obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

                    real_distance = pix_to_cm(obstacle_blob.h()-obstacle_blob.cy())
                    print('obstacle - cy', obstacle_blob.cy())
                    print('obstacle - h', obstacle_blob.h())
                    print('new pix', obstacle_blob.h()-obstacle_blob.cy())
                    print('real distance:', real_distance)
                    if real_distance > stop_cm:
                        print('almost there - moving')
                        steering = round((mid_blob.cx() - self.cam.w_centre) / (self.cam.w_centre * 2), 2)
                        print('obs steering', steering)
                        self.drive(speed, steering)
                        time.sleep_ms(100)
                        self.drive(0, 0) # Causes jerking but is more accurate!
                    else:
                        pan_angle = self.track_blob(obstacle_blob, pan=False)
                        while True:
                            print("pan angle", pan_angle)
                            if abs(pan_angle) > angle_thresh:
                                print("Done")
                                break
                            else:
                                print('moving')
                                if pan_angle < 0:
                                    self.servo.set_speed(0.1,-0.1)
                                    time.sleep(0.15)
                                else:
                                    self.servo.set_speed(-0.1,0.1)
                                    time.sleep(0.04)
                                self.servo.set_speed(0, 0)

                            pan_angle = self.track_blob(obstacle_blob, pan=True)

                        print('reached, bye bye')
                        print('real distance:', real_distance)
                        self.drive(0, 0)
                        break
                    obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

                # self.drive(0,0)

                # keep track of obstacle and if it's actually there then end
                # self.track_blob(obstacle_blob)
                break

            elif mid_blob: # no obstacle but mid line
                print("\nI found the mid blob! Yay")
                steering = round((mid_blob.cx() - self.cam.w_centre) / (self.cam.w_centre * 2), 2)
                print('steering', steering, 'mid_blobcx', mid_blob.cx())
                self.drive(speed, min(steering, 0.35))
                time.sleep_ms(100)
                # self.align_robot(mid_blob.cx(), self.mid_line_id)

            else: # no obstacle and no lane
                print("oh oh i don't see stuff")

                for i in range(5):
                    mid_blob = self.get_snap(self.mid_line_id)
                    if mid_blob:
                        break
                else:
                    print('still dont see stuff. bye bye')
                    break
            #     mid_blob = self.searching(mid_blob, self.mid_line_id)

        self.servo.set_speed(0,0)
        time.sleep(5)
        print("\n Okay, I will let you go")
        # self.servo.soft_reset()
        return


    def stage5(self, speed: float, bias: float) -> None:
        """
        Obstacle avoidance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.soft_reset()
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
            print("Moving pan angle to", pan_angle)
            # Move pan servo to track block
            self.servo.set_angle(pan_angle)

        return pan_angle # Pranathi added this


    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 20) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            found_idx = self.cam.find_blob(blobs, threshold_idx)
            if found_idx:
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                self.scan_direction *= -1


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
