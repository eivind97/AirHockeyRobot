import cv2
import numpy as np

from globfile import *
import pickle

class VideoStream():
    def __init__(self, src=0, width=640, height=480, framerate=30, buffersize=1):
        self.stream = cv2.VideoCapture(src) # source of where to get camera data

        #self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width) cannot be used when framerate is in use
        #self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height) cannot be used when framerate is in use
        self.stream.set(cv2.CAP_PROP_FPS, framerate) # framerate for video capture (speed)
        #self.stream.set(cv2.CAP_PROP_AUTOFOCUS, 0) # autofocus already done by camera
        self.stream.set(cv2.CAP_PROP_BUFFERSIZE, buffersize) # decide buffersize

        (self.grabbed, self.frame) = self.stream.read() # start videostream
        self.started = False
        self.read_lock = Lock() # create a thread

        self.current_time = None

        #Preset colour ranges
        #Blue
        #self.lower_range = np.array([99, 186, 130])
        #self.upper_range = np.array([159, 255, 255])

        #Green
        self.lower_range = np.array([41, 81, 71])
        self.upper_range = np.array(84, 255, 255)

        #Read pickle file and import camera calibration
        calib_result_pickle = pickle.load(open("camera_calib_pickle.p", "rb")) # camera calibration
        self.old_camera_matrix = calib_result_pickle["mtx"]
        self.optimal_camera_matrix = calib_result_pickle["optimal_camera_matrix"]
        self.distortion_matrix = calib_result_pickle["dist"]

        "ArUco dictionary and parameters"
        self.ARUCO_DICT = aruco.getPredefinedDictionray(aruco.DICT_ARUCO_ORIGINAL)
        self.arucoParameters = aruco.DetectorParameters_create()

        #ROI limits
        self.max_x = 0
        self.max_y = 0
        self.min_x = 10000
        self.min_y = 10000

        #Pixel conversion
        self.world_unit_px_conversion = 0
        self.world_unit_py_conversion = 0


    def start(self):
        # start camera and related threads
        if self.started:
            return None
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start() # start background thread
        return self


    def update(self):
        # Grab frames from camera
        while self.started:
            (grabbed, frame) = self.stream.read() # get frame
            self.read_lock.acquire() # get thread
            self.grabbed, self.frame = grabbed, frame
            self.current_time = datetime.now() # get time
            self.read_lock.release() # stop thread


    def undistort_camera(self):
        # Undistort camera with camera calibrations
        self.read_lock.acquire()
        frame = self.frame.copy()
        time = self.current_time
        self.read_lock.release()
        undistorted_frame = cv2.undistort(frame, self.old_camera_matrix, self.distortion_matrix, None, self.optimal_camera_matrix)
        return undistorted_frame, time


    def puck_color(self, min_hue, min_sat, min_val, max_hue, max_sat, max_val):
        # Getting HSV-color range for puck
        self.lower_range = np.array([min_hue, min_sat, min_val])
        self.upper_range = np.array([max_hue, max_sat, max_val])


    def robot_detection(self):
        frame, _ = self.undistort_camera()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #gray scale image
        corners, ids, rejected = aruco.detectMarkers(gray, dictionray=self.ARUCO_DICT, parameters=self.arucoParameters)

        if np.all(ids is not None):
            for i in range(len(ids)):

                if ids[i] == 4:
                    return True
        else:
            return False


    def corner_detection(self):
        #Detect Corners
        frame, _ = self.undistort_camera()

        #Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Find aruco markers in frame
        corners, ids, rejected = aruco.detectMarkers(gray, dictionary=self.ARUCO_DICT, parameters=self.arucoParameters)
        # Remove robot ID if found
        if np.all(ids is not None): # find all aruco markers
            remove_pos = -1
            for i in range(len(ids)): # for each aruco marker
                if ids[i] == 4: # if aruco marker id is 4
                    remove_pos = i

            if remove_pos != -1: # if remove_pos is not -1
                ids = np.delete(ids, remove_pos)  #remove id number 4
                corners.pop(remove_pos) # remove item at remove_pos value from table

        if np.all(ids is not None and len(ids) == 4): # if aruco markers have a value and table length is 4
            for id, corner in zip(ids, corners): # for ids and corners connected in joined tuple

                    # Playing area + offset
                if corner[0][0][0] < self.min_x:
                    self.min_x = int(corner[0][0][0]) - 35 # left side of table
                if corner[0][0][0] > self.max_x:
                    self.max_x = int(corner[0][0][0]) + 5 # right side of table
                if corner[0][0][1] < self.min_y:
                    self.min_y = int(corner[0][0][1]) - 6 # top of table
                if corner[0][0][1] > self.max_y:
                    self.max_y = int(corner[0][0][1]) + 5 # bottom of table

                 # Playing area
                #if corner[0][0][0] < self.min_x:
                 #   self.min_x = int(corner[0][0][0]) - 9
                #if corner[0][0][0] > self.max_x:
                 #   self.max_x = int(corner[0][0][0]) + 5
                #if corner[0][0][1] < self.min_y:
                 #   self.min_y = int(corner[0][0][1]) - 6
                #if corner[0][0][1] > self.max_y:
                  #  self.max_y = int(corner[0][0][1]) + 5

            if self.min_x < 0:
                self.min_x = 0
            if self.min_y < 0:
                self.min_y = 0

            roi_px = self.max_x - self.min_x
            roi_py = self.max_y - self.min_y

            #Relationship between pixel and millimeter
            self.world_unit_px_conversion = table_width / roi_px
            self.world_unit_py_conversion = table_height/ roi_py

            return _, True

        else:
            # Check which corners are missing
            aruco_verification = [False, False, False, False]

            if np.all(ids is not None):
                for i in range(len(ids)):
                    if ids[i] == 0:
                        aruco_verification[0] = True
                    if ids[i] == 1:
                        aruco_verification[1] = True
                    if ids[i] == 2:
                        aruco_verification[2] = True
                    if ids[i] == 3:
                        aruco_verification[3] = True

            return aruco_verification, False


    def region_of_interest(self):
        # Cropping frame to ROI
        frame, time = self.undistort_camera()
        roi = frame[self.min_y:self.max_y, self.min_x:self.max_x]
        return roi, time


    def pixel_to_mm(self, pixel_x, pixel_y):
        # Pixel units to millimeter
        pos_px = pixel_x * self.world_unit_px_conversion
        pos_py = pixel_y * self.world_unit_py_conversion
        pos_world_unit = Vec2d(pos_px, pos_py)
        return pos_world_unit


    def get_robot_coordinates_roi(self):
        # Get robot coordinates from ArUco ID 4 in ROi
        frame, _ = self.region_of_interest()

        # convert to gray scale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, dictionary=self.ARUCO_DICT, parameters=self.arucoParameters)

        center = -1
        if np.all(ids is not None):
            for id, corner in zip(ids, corners):
                if id == 4:
                    pixel_x = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4 # corner[][aruco marker][x]
                    pixel_y = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4 # corner[][aruco marker][y]

                    center = self.pixel_to_mm(pixel_x-20, pixel_y)

                    # Adding offset due to pusher height over playing surface
                    offset_x = pusher_height/camera_height * (table_center_x-center.x)
                    offset_y = pusher_height/camera_height * (table_center_y-center.y)
                    offset = Vec2d(offset_x, offset_y)

                    center = center + offset

        return center


    def get_robot_coordinates_full_fov(self):
        frame, _ = self.undistort_camera()

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, dictionray=self.ARUCO_DICT, parameters=self.arucoParameters)

        center = -1
        pixel_x = -1

        if np.all(ids is not None):
            for id, corner in zip(ids, corners):
                if id == 4:
                    pixel_x = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                    pixel_y = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4

                    center = Vec2d(pixel_x, pixel_y)

        return center


    def get_robot(self, frame, roi): # Function above do the same task, remove if not used elsewhere
        frame, _ = self.undistort_camera()

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected = aruco.detectMarkers(gray, dictionary=self.ARUCO_DICT, parameters=self.arucoParameters)

        #center = -1
        pixel_x = -1
        if np.all(ids is not None):
            for id, corner in zip(ids, corners):
                if id == 4:
                    pixel_x = (corner[0][0][0] + corner[0][1][0] + corner[0][2][0] + corner[0][3][0]) / 4
                    pixel_y = (corner[0][0][1] + corner[0][1][1] + corner[0][2][1] + corner[0][3][1]) / 4

                    Vec2d(pixel_x, pixel_y)
        return pixel_x


    def get_puck_coordinates(self):
        """Gets pucks' coordinates by masking puck with HSV color values"""
        roi, time = self.region_of_interest() # make roi and time into static variables

        # Convert from RHB to HSV
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # Mask puck with defined color range
        mask = cv2.inRange(hsv, self.lower_range, self.upper_range)

        # Increase mask size
        kernel = np.ones((5,5). np.uint8)
        dilation = cv2.dilate(mask, kernel)

        # Finding the contours
        contours, hierarchy = cv2.findContours(dilation, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Find biggest object with defined color
            obj = max(contours, key=cv2.contourArea)
            # Draw contour on object
            cv2.drawContours(roi, obj, -1, (0, 255, 0), 2)
            # Calculate center-coordinates of the object
            moments = cv2.moments(obj)

            pixel_x = int(moments["m10"] / moments["m00"]) - 20
            pixel_y = int(moments["m01"] / moments["m00"])

            # Convert from pixel to millimeters
            center_pos = self.pixel_to_mm(pixel_x, pixel_y)

        else:
            center_pos = -1

        return roi, time, center_pos


    def stop(self):
        self.started = False
        self.thread.join()


    def __exit__(self, exc_type, exc_value, traceback):
        self.stream.release()

