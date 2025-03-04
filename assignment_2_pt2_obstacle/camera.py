import sensor, time

class Cam(object):
    """
    The Cam class manages the camera sensor for image capturing, processing,
    and color tracking. It initializes the camera parameters and sets the color
    thresholds for blob detection.
    """

    def __init__(self, thresholds, gain = 10):
        """
        Initialise the Cam object by setting up camera parameters and
        configuring color thresholds.
        """
        # Configure camera settings
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.VGA)   # Set frame size to 640x480; add QVGA for increased fps but decreased resolution
        sensor.skip_frames(time=2000)   # Allow the camera to adjust to light levels

        # Both must be turned off for color tracking
        sensor.set_auto_gain(False, gain_db = gain)
        sensor.set_auto_whitebal(False)

        # Initialise sensor properties
        self.w_centre = sensor.width()/2
        self.h_centre = sensor.height()/2
        self.h_fov = 31.5
        self.v_fov = 21
        self.camera_elevation_angle = -11.5  # can measure and adjust this value
        self.clock = time.clock()

        # Define color tracking thresholds for Red, Green, Blue, and Yellow colors
        # Thresholds are in the order of (L Min, L Max, A Min, A Max, B Min, B Max)
        self.thresholds = thresholds


    def get_blobs(self, angle = 0, area_thresh=60, pix_thresh=60) -> tuple:
        """
        Capture an image and detect color blobs based on predefined thresholds.
        Args:
            angle (float): pan angle for image rotation correction. Defaults to 0.
        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """
        img = sensor.snapshot()
        img.rotation_corr(z_rotation=angle)
        blobs = img.find_blobs(self.thresholds,pixels_threshold=pix_thresh,area_threshold=area_thresh)

        # blobs = img.find_blobs(self.thresholds,pixels_threshold=1000,area_threshold=area_thresh)

        return blobs, img


    def get_blobs_bottom(self, angle = 0) -> tuple:
        """
        Capture an image and detect colour blobs based on predefined thresholds.
        Region of interest is set to the bottom 2/3 of the image.
        Args:
            angle (float): pan angle for image rotation correction. Defaults to 0.
        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """
        img = sensor.snapshot()
        img.rotation_corr(z_rotation=angle)

        blobs = img.find_blobs(self.thresholds,pixels_threshold=150,area_threshold=150,
                               roi=(1,int(sensor.height()/3),
                                    int(sensor.width()),int(2*sensor.height()/3)))

        return blobs, img


    def get_biggest_blob(self, blobs):
        """
        Identify and return the largest blob from a list of detected blobs.

        Args:
            blobs (list): List of detected blobs.

        Returns:
            big_blob (blob): The biggest blob from list - see OpenMV docs for blob class.
        """
        max_pixels = 0
        big_blob = None

        for blob in blobs:
            # Update the big blob if the current blob has more pixels
            if blob.pixels() > max_pixels:
                max_pixels = blob.pixels()
                big_blob = blob

        return big_blob


    def get_blob_colours(self, blobs) -> int:
        """
        Returns the binary code (as int) of thresholds met by each blob

        Args:
            blobs (list): List of detected blobs.

        Returns:
            colours (list): List of binary codes (as int) of thresholds met by each element in blobs.
        """
        colours = []

        for blob in blobs:
            colours.append(blob[8])

        return colours


    def find_blob(self, blobs, threshold_idx: int):
        """
        Finds the first blob in blobs that was detected using a specified threshold

        Args:
            blobs (list): List of detected blobs.
            threshold_idx (int): Index along self.thresholds.

        Returns:
            found_idx (int): Index along blobs for the first blob that was detected using self.thresholds(threshold_idx)
        """
        colours = self.get_blob_colours(blobs)

        for found_idx, colour in enumerate(colours):
            if colour == pow(2, threshold_idx):
                return found_idx

        return None


if __name__ == "__main__":
    #
    # Blob threshold tester
    #
    # Use this code to determine colour tracking thresholds

    import sensor
    import math

    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.VGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_whitebal(False)  # must be turned off for colour tracking

    # Change gain here to work with the lighting conditions you have
    sensor.set_auto_gain(False, gain_db = 10)  # can change the gain depending to the brightnes;
    # make sure you pass the gain you tested it when initialising the Cam object in your script


    # Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
    # The below thresholds track in general red/green things. You may wish to tune them...
    thresholds = [
    #Pranathi's living room table - gain 10 with lamp
    # (50, 63, 20, 33, -15, 7), # red
    # (63, 69, 20, 38, -16, 5), # red 2 - phone as well
    # (70, 76, 27, 41, -18, 2), # red 3
    # (56, 77, 14, 48, -15, 12), # red 4
    # (59, 65, 23, 39, -15, 12), # red 5
    # (65, 71, 23, 39, -15, 12), # red 6
    # (56, 68, 24, 41, -15, 12), # red 7
    # (37, 49, 12, 21, -34, -19), # purple
    # (44, 56, 15, 29, -32, -8), # purple 2
    # (57, 69, -17, -4, -26, -11) # light blue
    # (64, 80, -15, -4, -27, -6) # light blue 2
    # (73, 77, -8, 2, -30, -16), # light blue 3
    # (44, 63, -9, 10, -15, 12), # green paper obstacle
     # (7, 18, -11, 9, -15, 7) # black obstacle


    # Pranathi's bedroom floor - gain 10 - lamp + phone
    # (34, 43, 14, 38, 0, 32), # red 1
    (39, 56, 16, 38, -6, 20), # red 2
    # (47, 57, -23, -7, -20, -3), # light blue 1
    # (40, 56, -14, 0, -26, -13), # light blue 2
    (54, 71, -14, 1, -25, -15), # light blue 3
    # (8, 23, -2, 18, -18, 1), # purple 1
    (13, 28, 8, 25, -25, -7), # purple 2
    # (9, 42, -11, 5, 13, 41) # obstacle yellow 1
    (16, 47, -5, 12, 14, 37), # obstacle yellow 2

    # Rachael's thresholds in Skempton
    # (41, 46, 27, 42, 7, 35), # red
    # (42, 54, -20, -1, -29, -6), # blue
    # (19, 29, -8, 31, -22, 13), # purple

    ]

    angle = 0 # Set pan angle for rotation correction


    # Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
    # returned by "find_blobs" below. "roi" sets the region of interest of the image in which to find
    # blobs (x, y, width, height). Currently set to look for blobs in the bottom 2/3 of the image.
    # time.sleep(5)
    while True:
        img = sensor.snapshot()
        img.rotation_corr(z_rotation=angle)
        # print('sztart')
        for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150):
            img.draw_rectangle(blob.rect())
            if blob.code() == 1:
                # print(blob)
                print(blob.cx(), blob.cy(), blob.h())
            # print(blob)
            # img.draw_cross(blob.cx(), blob.cy())
            # img.draw_keypoints(
            #    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
            # )
            img.draw_string(blob.cx(), blob.cy(), str(int(blob.code())), scale=2)
        blobs = img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150)
        furthest_blob = sorted(blobs, key=lambda b: b.cy())
        # print('end')
        # break

    # # Try uncommenting these lines to see what happens
    #         if blob.elongation() > 0.5:
    #            img.draw_edges(blob.min_corners(), color=(255, 0, 0))
    #            img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
    #            img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
    #

    # Try uncommenting these lines to see what happens
           #  img.draw_cross(blob.cx(), blob.cy())
           # # Note - the blob rotation is unique to 0-180 only.
           #  img.draw_keypoints(
           #     [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
           #  )
    #
