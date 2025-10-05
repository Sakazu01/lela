#!/usr/bin/env python3
import cv2
import numpy as np

class HSVConfig:
    """
    Class untuk konfigurasi HSV range menggunakan trackbar
    Memudahkan tuning parameter deteksi warna secara real-time
    """
    
    def __init__(self):
        # Buat window untuk trackbar
        cv2.namedWindow("HSV Config")
        
        # Red trackbars (range 1: 0-10 derajat)
        cv2.createTrackbar("R_Low_H", "HSV Config", 0, 179, lambda x: None)
        cv2.createTrackbar("R_Low_S", "HSV Config", 100, 255, lambda x: None)
        cv2.createTrackbar("R_Low_V", "HSV Config", 100, 255, lambda x: None)
        cv2.createTrackbar("R_Up_H", "HSV Config", 10, 179, lambda x: None)
        cv2.createTrackbar("R_Up_S", "HSV Config", 255, 255, lambda x: None)
        cv2.createTrackbar("R_Up_V", "HSV Config", 255, 255, lambda x: None)
        
        # Blue trackbars
        cv2.createTrackbar("B_Low_H", "HSV Config", 100, 179, lambda x: None)
        cv2.createTrackbar("B_Low_S", "HSV Config", 100, 255, lambda x: None)
        cv2.createTrackbar("B_Low_V", "HSV Config", 100, 255, lambda x: None)
        cv2.createTrackbar("B_Up_H", "HSV Config", 130, 179, lambda x: None)
        cv2.createTrackbar("B_Up_S", "HSV Config", 255, 255, lambda x: None)
        cv2.createTrackbar("B_Up_V", "HSV Config", 255, 255, lambda x: None)
        
    def get_hsv(self):
        """
        Ambil nilai HSV dari trackbar
        
        Returns:
            Dictionary dengan HSV ranges untuk red dan blue
        """
        return {
            'red_low1': np.array([
                cv2.getTrackbarPos("R_Low_H", "HSV Config"), 
                cv2.getTrackbarPos("R_Low_S", "HSV Config"), 
                cv2.getTrackbarPos("R_Low_V", "HSV Config")
            ]),
            'red_up1': np.array([
                cv2.getTrackbarPos("R_Up_H", "HSV Config"), 
                cv2.getTrackbarPos("R_Up_S", "HSV Config"), 
                cv2.getTrackbarPos("R_Up_V", "HSV Config")
            ]),
            'red_low2': np.array([170, 100, 100]),  # Range 2 untuk red (170-180)
            'red_up2': np.array([180, 255, 255]),
            'blue_low': np.array([
                cv2.getTrackbarPos("B_Low_H", "HSV Config"), 
                cv2.getTrackbarPos("B_Low_S", "HSV Config"), 
                cv2.getTrackbarPos("B_Low_V", "HSV Config")
            ]),
            'blue_up': np.array([
                cv2.getTrackbarPos("B_Up_H", "HSV Config"), 
                cv2.getTrackbarPos("B_Up_S", "HSV Config"), 
                cv2.getTrackbarPos("B_Up_V", "HSV Config")
            ])
        }
    
    def destroy(self):
        """Destroy window"""
        cv2.destroyWindow("HSV Config")
