import cv2
import numpy as np

class HSVConfig:
    def __init__(self):
        self.window_name = "HSV Controls"
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 600, 400)

        def nothing(x):
            pass

        # Red 1
        cv2.createTrackbar("Red1 H Min", self.window_name, 0, 179, nothing)
        cv2.createTrackbar("Red1 S Min", self.window_name, 70, 255, nothing)
        cv2.createTrackbar("Red1 V Min", self.window_name, 50, 255, nothing)
        cv2.createTrackbar("Red1 H Max", self.window_name, 20, 179, nothing)
        cv2.createTrackbar("Red1 S Max", self.window_name, 230, 255, nothing)
        cv2.createTrackbar("Red1 V Max", self.window_name, 255, 255, nothing)
        
        # Red 2 
        cv2.createTrackbar("Red2 H Min", self.window_name, 170, 179, nothing)
        cv2.createTrackbar("Red2 S Min", self.window_name, 70, 255, nothing)
        cv2.createTrackbar("Red2 V Min", self.window_name, 50, 255, nothing)
        cv2.createTrackbar("Red2 H Max", self.window_name, 179, 179, nothing)
        cv2.createTrackbar("Red2 S Max", self.window_name, 230, 255, nothing)
        cv2.createTrackbar("Red2 V Max", self.window_name, 255, 255, nothing)
        
        # Blue
        cv2.createTrackbar("Blue H Min", self.window_name, 100, 179, nothing)
        cv2.createTrackbar("Blue S Min", self.window_name, 50, 255, nothing)
        cv2.createTrackbar("Blue V Min", self.window_name, 50, 255, nothing)
        cv2.createTrackbar("Blue H Max", self.window_name, 130, 179, nothing)
        cv2.createTrackbar("Blue S Max", self.window_name, 255, 255, nothing)
        cv2.createTrackbar("Blue V Max", self.window_name, 255, 255, nothing)

    def get_hsv(self):
        values = {
            'red_low1': np.array([
                cv2.getTrackbarPos("Red1 H Min", self.window_name),
                cv2.getTrackbarPos("Red1 S Min", self.window_name),
                cv2.getTrackbarPos("Red1 V Min", self.window_name)
            ]),
            'red_up1': np.array([
                cv2.getTrackbarPos("Red1 H Max", self.window_name),
                cv2.getTrackbarPos("Red1 S Max", self.window_name),
                cv2.getTrackbarPos("Red1 V Max", self.window_name)
            ]),
            'red_low2': np.array([
                cv2.getTrackbarPos("Red2 H Min", self.window_name),
                cv2.getTrackbarPos("Red2 S Min", self.window_name),
                cv2.getTrackbarPos("Red2 V Min", self.window_name)
            ]),
            'red_up2': np.array([
                cv2.getTrackbarPos("Red2 H Max", self.window_name),
                cv2.getTrackbarPos("Red2 S Max", self.window_name),
                cv2.getTrackbarPos("Red2 V Max", self.window_name)
            ]),
            'blue_low': np.array([
                cv2.getTrackbarPos("Blue H Min", self.window_name),
                cv2.getTrackbarPos("Blue S Min", self.window_name),
                cv2.getTrackbarPos("Blue V Min", self.window_name)
            ]),
            'blue_up': np.array([
                cv2.getTrackbarPos("Blue H Max", self.window_name),
                cv2.getTrackbarPos("Blue S Max", self.window_name),
                cv2.getTrackbarPos("Blue V Max", self.window_name)
            ])
        }
        return values
