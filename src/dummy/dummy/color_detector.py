#!/usr/bin/env python3
import cv2
import numpy as np

class ColorDetector:
    """
    Class untuk mendeteksi objek berdasarkan warna (merah dan biru)
    Menggunakan HSV color space dan morphological operations
    """
    
    def __init__(self, min_area=5000, max_area=100000):
        # Morphological kernels untuk noise reduction
        self.kernel_small = np.ones((3, 3), np.uint8)
        self.kernel_medium = np.ones((5, 5), np.uint8)
        self.kernel_large = np.ones((7, 7), np.uint8)
        
        # Parameter deteksi
        self.min_area = min_area
        self.max_area = max_area
        
    def create_mask(self, hsv_image, color_type, hsv_ranges):
        """
        Membuat mask berdasarkan range HSV
        
        Args:
            hsv_image: Image dalam HSV color space
            color_type: 'red' atau 'blue'
            hsv_ranges: Dictionary berisi HSV range values
        
        Returns:
            Binary mask
        """
        if color_type == 'red':
            # Red memiliki 2 range karena melewati 0° di HSV
            mask1 = cv2.inRange(hsv_image, hsv_ranges['red_low1'], hsv_ranges['red_up1'])
            mask2 = cv2.inRange(hsv_image, hsv_ranges['red_low2'], hsv_ranges['red_up2'])
            mask = cv2.bitwise_or(mask1, mask2)
        elif color_type == 'blue':
            mask = cv2.inRange(hsv_image, hsv_ranges['blue_low'], hsv_ranges['blue_up'])
        else:
            return None
        
        return mask
    
    def clean_mask(self, mask):
        """
        Membersihkan mask dengan morphological operations
        
        Args:
            mask: Binary mask
        
        Returns:
            Cleaned binary mask
        """
        # Hilangkan noise kecil
        mask = cv2.medianBlur(mask, 5)
        
        # Opening: hilangkan noise
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_small, iterations=2)
        
        # Closing: isi gaps
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, self.kernel_medium, iterations=2)
        
        # Threshold final
        _, final = cv2.threshold(closing, 127, 255, cv2.THRESH_BINARY)
        
        return final
    
    def detect_objects(self, mask, color_name):
        """
        Deteksi objek dari mask
        
        Args:
            mask: Binary mask
            color_name: Nama warna untuk labeling
        
        Returns:
            List of detected objects dengan info detail
        """
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if area < self.min_area or area > self.max_area:
                continue
            
            # Bounding rectangle
            x, y, w, h = cv2.boundingRect(contour)
            
            # Centroid
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = x + w // 2, y + h // 2
            
            detected_objects.append({
                'contour': contour,
                'area': area,
                'center': (cx, cy),
                'bbox': (x, y, w, h),
                'color': color_name
            })
        
        return detected_objects
    
    def draw_detections(self, image, objects):
        """
        Gambar hasil deteksi di image
        
        Args:
            image: Original image
            objects: List of detected objects
        
        Returns:
            Image dengan visualization
        """
        result = image.copy()
        
        for obj in objects:
            color = (0, 0, 255) if obj['color'] == 'red' else (255, 0, 0)  # BGR
            
            # Draw bounding box
            x, y, w, h = obj['bbox']
            cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
            
            # Draw center
            cx, cy = obj['center']
            cv2.circle(result, (cx, cy), 5, color, -1)
            
            # Label
            label = f"{obj['color'].upper()} ({int(obj['area'])})"
            cv2.putText(result, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        return result
    
    def process_frame(self, frame, hsv_ranges):
        """
        Process frame lengkap: deteksi red dan blue
        
        Args:
            frame: BGR image
            hsv_ranges: Dictionary HSV ranges
        
        Returns:
            result_image, red_mask, blue_mask, all_objects
        """
        # Convert ke HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create masks
        red_mask = self.create_mask(hsv, 'red', hsv_ranges)
        blue_mask = self.create_mask(hsv, 'blue', hsv_ranges)
        
        # Clean masks
        red_mask_clean = self.clean_mask(red_mask)
        blue_mask_clean = self.clean_mask(blue_mask)
        
        # Detect objects
        red_objects = self.detect_objects(red_mask_clean, 'red')
        blue_objects = self.detect_objects(blue_mask_clean, 'blue')
        
        all_objects = red_objects + blue_objects
        
        # Draw results
        result_image = self.draw_detections(frame, all_objects)
        
        return result_image, red_mask_clean, blue_mask_clean, all_objects
