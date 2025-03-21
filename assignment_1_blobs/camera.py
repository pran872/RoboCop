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
        sensor.set_framesize(sensor.VGA)   # Set frame size to 640x480
        sensor.skip_frames(time=2000)   # Allow the camera to adjust to light levels

        # Both must be turned off for color tracking
        sensor.set_auto_gain(False, gain_db = gain)
        sensor.set_auto_whitebal(False)

        # Initialise sensor properties
        self.w_centre = sensor.width()/2
        self.h_centre = sensor.height()/2
        self.h_fov = 70.8
        self.v_fov = 55.6
        self.camera_elevation_angle = -11
        self.clock = time.clock()

        # Define color tracking thresholds for Red, Green, Blue, and Yellow colors
        # Thresholds are in the order of (L Min, L Max, A Min, A Max, B Min, B Max)
        self.thresholds = thresholds


    def get_blobs(self, angle = 0) -> tuple:
        """
        Capture an image and detect color blobs based on predefined thresholds.

        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """
        img = sensor.snapshot()
        img.rotation_corr(z_rotation=angle)
        # blobs = img.find_blobs(self.thresholds,pixels_threshold=60,area_threshold=60)

        blobs = img.find_blobs(self.thresholds,pixels_threshold=150,area_threshold=150)

        return blobs, img


    def get_blobs_bottom(self, angle = 0) -> tuple:
        """
        Capture an image and detect colour blobs based on predefined thresholds.
        Region of interest is set to the bottom 2/3 of the image.

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
    sensor.set_auto_gain(False, gain_db = 10)  # must be turned off for color tracking
    #

    # Color Tracking Thresholds (L Min, L Max, A Min, A Max, B Min, B Max)
    # The below thresholds track in general red/green things. You may wish to tune them...
    thresholds = [

    (34, 46, -28, -15, 2, 24), # green
    # (34, 46, -28, -15, 2, 24) # green
    # (39, 50, -4, 13, -61, -38) #blue
    # (75, 89, -23, 4, 20, 68) # new yellow
    # (43, 49, 40, 59, 13, 41) # red
     # (34, 46,-28, -15, 2, 24), #green
     # (38, 57, 39, 66, 5, 42) #red new
    # (52, 61, 23, 39, 19, 50) # orange
    # (18, 90, 13, 30, -20, 8) # old purple
    # (28, 36, 8, 25, -16, 2) # purple
    # (60, 77, -40, -18, 4, 53) # light green

    # tuesday phd area new thresholds
    # (56, 62, -20, 0, -27, 0), #green
    # (70, 74, -8, 3, -44, -30), #blue
    # (65, 72, -10, 4, 8, 37), # yellow
    # (76, 81, 26, 40, -31, -13), #red from blue
    # (69, 100, 20, 41, -26, -9) # red from yellow
    # (36, 54, 9, 26, -30, -10), #old good purple
    # (33, 60, 10, 29, -39, 5) # purple
    # (79, 87, 18, 30, -24, -9), #orange
    # (56, 64, -21, -1, -9, 15) #lightgreen

    (24, 32, 11, 31, -20, 0),
    (38, 50, -6, 10, -51, -23),
    (57, 72, -49, -18, 14, 61),

    ]

    angle = 0 # Set pan angle for rotation correction


    # Only blobs that with more pixels than "pixel_threshold" and more area than "area_threshold" are
    # returned by "find_blobs" below. "roi" sets the region of interest of the image in which to find
    # blobs (x, y, width, height). Currently set to look for blobs in the bottom 2/3 of the image.
    while True:
        img = sensor.snapshot()
        img.rotation_corr(z_rotation=angle)

        for blob in img.find_blobs(thresholds, pixels_threshold=150, area_threshold=150):
            img.draw_rectangle(blob.rect())

    # Try uncommenting these lines to see what happens
            # if blob.elongation() > 0.5:
            #    img.draw_edges(blob.min_corners(), color=(255, 0, 0))
            #    img.draw_line(blob.major_axis_line(), color=(0, 255, 0))
            #    img.draw_line(blob.minor_axis_line(), color=(0, 0, 255))
    #

    # Try uncommenting these lines to see what happens
            # img.draw_cross(blob.cx(), blob.cy())
            # # Note - the blob rotation is unique to 0-180 only.
            # img.draw_keypoints(
            #    [(blob.cx(), blob.cy(), int(math.degrees(blob.rotation())))], size=20
            # )
    #
