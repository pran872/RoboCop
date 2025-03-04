from servos import *
from camera import *
from pid_control import PID
import time

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain, p_steer, d_steer, p=0.18, i=0, d=0, imax=0.01):
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
        self.PID_steering = PID(p_steer, 0, d_steer, imax=0.0)

        # Blob IDs
        self.mid_line_id = 1
        self.obstacle_id = 8
        self.l_line_id = 2
        self.r_line_id = 4

        self.scan_direction = 1
        self.thresholds = thresholds


    def get_snap(self, col, debug_mode=False, area_thresh=60, pix_thresh=60, show_one_blob=False, closest=False):
        blobs=[b for b in self.cam.get_blobs(area_thresh=area_thresh, pix_thresh=pix_thresh)[0] if b.code()==col]
        if debug_mode:
            img = sensor.snapshot()

            for blob in img.find_blobs(self.thresholds, pixels_threshold=pix_thresh, area_threshold=area_thresh):
                if show_one_blob and blob.code()!=col:
                    next
                else:
                    print(blob)
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(), blob.cy(), str(int(blob.code())), scale=2)

        if blobs and closest:
            closest_blob = sorted(blobs, key=lambda b: b.cy())
            return closest_blob[-1]

        return self.cam.get_biggest_blob(blobs)


    def stage1(self, speed=0.1) -> None: # Previously known as stage1_only_mid
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        found_mid = False
        self.servo.set_angle(0)
        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                found_mid = True
                pixel_error = mid_blob.cx() - self.cam.w_centre
                steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
                self.drive(speed, steering)

            else: # Cannot find any blob
                if found_mid: # But found middle blob before, then end the function - we're done
                    print("\n I did it. Hooray!")
                    self.drive(0, 0)
                    break

                else: # Look for the blob
                    print("yoohoo, dont see anything")

        self.servo.set_angle(0)
        self.servo.soft_reset()

    def stage1_robust(self, speed=0.1) -> None:
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.set_angle(0)
        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            right_blob = self.get_snap(self.r_line_id, closest=True) # get the closest blob
            left_blob = self.get_snap(self.l_line_id, closest=True) # get the closest blob


            if mid_blob: # Found the middle blob; moving to it
                print("\nI found the mid blob! Yay")
                pixel_error = mid_blob.cx() - self.cam.w_centre

            elif right_blob:
                print("\nI found right blob instead. Yay?")
                pixel_error = right_blob.cx() + 75

            elif left_blob:
                print("\nI found left blob instead. Yay?")
                pixel_error = left_blob.cx() - 600

            else: # Cannot find any blob
                print("\nScanning for blob")

                self.drive(0, 0)
                mid_blob = self.scan_for_blob(0, limit=30, step=2)

                if mid_blob:
                    print("Found the blob. Trying again")
                    pixel_error = mid_blob.cx() - self.cam.w_centre
                    steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
                    self.drive(speed, steering)
                    self.servo.set_angle(0)
                    continue

                else:
                    print("\n I did it. Hooray!")
                    break

            steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
            self.drive(speed, steering)

        self.servo.set_angle(0)
        self.servo.soft_reset()


    def stage2(self, speed=0.1) -> None:
        """
        Obstacle detection algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """

        found_mid = False
        self.servo.set_angle(0)
        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

            if obstacle_blob:
                print("\nOh oh there's an obstacle. I'm stopping.")
                print("But I'm watching you")
                self.drive(0,0)
                self.track_blob(obstacle_blob)  # keep track of obstacle
                break

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                found_mid = True
                pixel_error = mid_blob.cx() - self.cam.w_centre
                steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
                self.drive(speed, steering)

            else: # Cannot find any blob
                if found_mid: # But found middle blob before, then end the function - we're done
                    print("\n I did it. Hooray!")
                    self.drive(0, 0)
                    break

                else: # Look for the blob
                    print("yoohoo, dont see anything")

        print(f"You are at {self.servo.pan_pos} pos")
        time.sleep(5)
        print("\nOkay, I will let you go")
        self.servo.set_angle(0)
        self.servo.soft_reset()
        return

    def stage2_robust(self, speed=0.1) -> None:
        """
        Obstacle detection algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.set_angle(0)
        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)
            right_blob = self.get_snap(self.r_line_id, closest=True) # get the closest blob
            left_blob = self.get_snap(self.l_line_id, closest=True) # get the closest blob

            if obstacle_blob:
                print("\nOh oh there's an obstacle. I'm stopping.")
                print("But I'm watching you")
                self.drive(0,0)
                self.track_blob(obstacle_blob)  # keep track of obstacle
                break

            if mid_blob: # Found the middle blob; moving to it
                print("\nI found the mid blob! Yay")
                pixel_error = mid_blob.cx() - self.cam.w_centre

            elif right_blob:
                print("\nI found right blob instead. Yay?")
                pixel_error = right_blob.cx() + 75

            elif left_blob:
                print("\nI found left blob instead. Yay?")
                pixel_error = left_blob.cx() - 600

            else: # Cannot find any blob
                print("\nScanning for blob")

                self.drive(0, 0)
                mid_blob = self.scan_for_blob(0, limit=30, step=2)

                if mid_blob:
                    print("Found the blob. Trying again")
                    pixel_error = mid_blob.cx() - self.cam.w_centre
                    steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
                    self.drive(speed, steering)
                    self.servo.set_angle(0)
                    continue

                else:
                    print("\n I did it. Hooray!")
                    break

            steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
            self.drive(speed, steering)

        print(f"You are at {self.servo.pan_pos} pos")
        time.sleep(5)
        print("\nOkay, I will let you go")
        self.servo.set_angle(0)
        self.servo.soft_reset()
        return


    def stage3(self, speed=0.1, stop_cm=10) -> None:
        """
        Obstacle distance algorithm - FOLLOWS MID BLOB!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        stop_cm += 5
        print("STOP CM:", stop_cm-5, stop_cm)
        # pix_to_cm = lambda x: 0.000105*x**2 + -0.104068*x + 25.931873  # curve 1
        # pix_to_cm = lambda x: 0.000076*x**2 + -0.086957*x + 32.820661  # curve 2
        pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524  # curve 3

        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

            if obstacle_blob: # obstacle detected
                print("\nOh oh there's an obstacle. I'm stopping but I'm looking at you")
                self.drive(0,0)
                real_distance = pix_to_cm((obstacle_blob.h()/2)+obstacle_blob.cy())  # for curve 3


                # print('new pix', obstacle_blob.h()-obstacle_blob.cy())  # for curve 1
                print('new pix', (obstacle_blob.h()/2)+obstacle_blob.cy())  # for curve 3
                print('real distance', real_distance)


                if real_distance < stop_cm or abs(real_distance - stop_cm) < 2:
                    print('reached, bye bye')
                    self.drive(0, 0)
                    self.track_blob(obstacle_blob)
                    break

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                found_mid = True
                pixel_error = mid_blob.cx() - self.cam.w_centre
                steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
                self.drive(speed, steering)

            else: # Cannot find any blob
                if found_mid: # But found middle blob before, then end the function - we're done
                    print("\n I did it. Hooray!")
                    self.drive(0, 0)
                    break

                else: # Look for the blob
                    print("yoohoo, dont see anything")

        self.servo.set_speed(0,0)
        time.sleep(5)
        print("\n Okay, I will let you go")
        self.servo.soft_reset()
        return

    def track_and_move(self, target_blob, stop_angle, speed=0.1):

        while True:
            pan_angle, pid_error = self.track_blob(target_blob)
            print("Obstacle angle:", pan_angle, "PID error:", pid_error)
            if abs(pan_angle) > stop_angle:
                break

            turn = 1 if pan_angle < 0 else -1 # if pan angle is -ve turn is +ve
            turn_bias = turn * (pan_angle * turn - stop_angle + pid_error) / 90
            print("form", turn * (pan_angle * turn - stop_angle + pid_error))
            print("Turn bias", turn_bias)
            self.drive(speed, turn_bias)
            time.sleep_ms(1000)

        print("done")
        self.drive(0, 0)



    def stage4(self, speed=0.1, stop_cm=10, stop_angle=90) -> None:
        """
        Obstacle distance algorithm - FOLLOWS MID BLOB!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        # if stop_cm
        stop_cm += 5
        pix_to_cm = lambda x: 0.000101*x**2 + -0.100282*x + 34.739524  # curve 3

        print("\nI am good to go. Starting now!")

        while True:
            mid_blob = self.get_snap(self.mid_line_id) # get biggest blob of the mid color
            obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

            if obstacle_blob: # obstacle detected
                print("\nOh oh there's an obstacle. I'm stopping but I'm looking at you")
                self.drive(0,0)
                real_distance = pix_to_cm((obstacle_blob.h()/2)+obstacle_blob.cy())  # for curve 3

                # print('new pix', obstacle_blob.h()-obstacle_blob.cy())  # for curve 1
                # print('new pix', (obstacle_blob.h()/2)+obstacle_blob.cy())  # for curve 3
                # print('real distance', real_distance)

                if real_distance < stop_cm or abs(real_distance - stop_cm) < 2:
                    print('\nReached distance')
                    self.drive(0, 0)

                    self.track_and_move(obstacle_blob, stop_angle, speed=0.1)
                    break

            if mid_blob: # Found the middle blob - then move to it
                print("\nI found the mid blob! Yay")
                found_mid = True
                pixel_error = mid_blob.cx() - self.cam.w_centre
                steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
                self.drive(speed, steering)

            else: # Cannot find any blob
                if found_mid: # But found middle blob before, then end the function - we're done
                    print("\n I did it. Hooray!")
                    self.drive(0, 0)
                    break

                else: # Look for the blob
                    print("yoohoo, dont see anything")

        self.servo.set_speed(0,0)
        # time.sleep(5)
        # print("\n Okay, I will let you go")
        # self.servo.soft_reset()
        return


    def stage4_old(self, speed=0.1, stop_cm=10, angle_thresh=20) -> None:
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
                            # steering = round((mid_blob.cx() - self.cam.w_centre) / (self.cam.w_centre * 2), 2)
                            # print('obs steering', steering)
                            # self.drive(speed, steering)
                            pixel_error = mid_blob.cx() - self.cam.w_centre
                            steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
                            self.drive(speed, steering)
                            time.sleep_ms(100)
                            self.drive(0, 0) # Causes jerking but is more accurate!
                        else:
                            pan_angle, _ = self.track_blob(obstacle_blob, pan=False)
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

                                pan_angle, _ = self.track_blob(obstacle_blob, pan=True)

                            print('reached, bye bye')
                            print('real distance:', real_distance)
                            print('pan angle:', pan_angle)
                            self.drive(0, 0)
                            break
                        obstacle_blob = self.get_snap(self.obstacle_id, pix_thresh=2000)

                    # self.drive(0,0)

                    # keep track of obstacle and if it's actually there then end
                    # self.track_blob(obstacle_blob)
                    break

                elif mid_blob: # no obstacle but mid line
                    print("\nI found the mid blob! Yay")
                    # steering = round((mid_blob.cx() - self.cam.w_centre) / (self.cam.w_centre * 2), 2)
                    # print('steering', steering, 'mid_blobcx', mid_blob.cx())
                    # self.drive(speed, min(steering, 0.35))

                    pixel_error = mid_blob.cx() - self.cam.w_centre
                    steering = self.PID_steering.get_pid(pixel_error/sensor.width(), scaler=1.0)
                    self.drive(speed, steering)
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

        return pan_angle, pid_error # Pranathi added this


    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 20) -> None:
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
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)
            # print('new pan angle:', new_pan_angle)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            # blobs, _ = self.cam.get_blobs(self.servo.pan_pos)
            blobs, _ = self.cam.get_blobs()
            found_idx = self.cam.find_blob(blobs, threshold_idx)
            if found_idx is not None:
                print("Scanned and found")
                return blobs[found_idx]

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                count += 1
                self.scan_direction *= -1

                if count > 1:
                    print("Scanned and not found")
                    return None

    def scan_for_multiple_blobs(self, threshold_idxs: list, step = 2, limit = 20) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        if not isinstance(threshold_idxs, list):
            threshold_idxs = [threshold_idxs]

        blob_codes = [int(2**thresh) for thresh in threshold_idxs]
        print('blob_codes', blob_codes)

        count = 0
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)
            print('new pan angle:', new_pan_angle)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs()
            for blob in blobs:
                if blob.code() in blob_codes:
                    print("Scanned and found:", blob.code())
                    return blob

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                count += 1
                self.scan_direction *= -1

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
