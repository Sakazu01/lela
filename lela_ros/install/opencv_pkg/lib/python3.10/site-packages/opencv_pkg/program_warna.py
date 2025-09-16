import cv2
import numpy as np
from opencv_pkg.hsv_config import HSVConfig

class tarp:
    def __init__(self):
        # Initialize morphological kernels
        self.kernel_small = np.ones((3, 3), np.uint8)
        self.kernel_medium = np.ones((5, 5), np.uint8)
        self.kernel_large = np.ones((7, 7), np.uint8)
        self.kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        
        # Detection parameters
        self.min_area = 5000
        self.max_area = 100000  # Add maximum area to filter out very large objects
        
    def create_mask(self, hsv_image, color_type, hsv_values):
        """Create color mask based on HSV values"""
        if color_type == 'red':
            # Red color spans across 0° in HSV, so we need two ranges
            mask1 = cv2.inRange(hsv_image, hsv_values['red_low1'], hsv_values['red_up1'])
            mask2 = cv2.inRange(hsv_image, hsv_values['red_low2'], hsv_values['red_up2'])
            mask = cv2.bitwise_or(mask1, mask2)
        elif color_type == 'blue':
            mask = cv2.inRange(hsv_image, hsv_values['blue_low'], hsv_values['blue_up'])
        else:
            return None
        return mask

    def clean_mask(self, mask):
        """Clean the mask using morphological operations and filtering"""
        # Reduce noise with median blur
        mask = cv2.medianBlur(mask, 5)
        
        # Remove small noise with opening
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_small, iterations=2)
        
        # Fill gaps with closing
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, self.kernel_medium, iterations=2)
        
        # Smooth edges with elliptical kernel
        smooth = cv2.morphologyEx(closing, cv2.MORPH_OPEN, self.kernel_ellipse, iterations=1)
        
        # Final closing to ensure solid objects
        final = cv2.morphologyEx(smooth, cv2.MORPH_CLOSE, self.kernel_large, iterations=1)
        
        # Apply Gaussian blur and threshold
        final = cv2.GaussianBlur(final, (3, 3), 0)
        _, final = cv2.threshold(final, 127, 255, cv2.THRESH_BINARY)
        
        return final
    
    def detect_objects(self, mask, color_name):
        """Detect objects in the cleaned mask"""
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detected_objects = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area
            if area < self.min_area or area > self.max_area:
                continue
            
            # Approximate contour to smooth it
            epsilon = 0.02 * cv2.arcLength(contour, True)
            smooth_contour = cv2.approxPolyDP(contour, epsilon, True)
            
            # Get bounding rectangle
            x, y, w, h = cv2.boundingRect(smooth_contour)
            aspect_ratio = w / h
            
            # Filter by aspect ratio to avoid very thin or very wide objects
            if aspect_ratio > 5 or aspect_ratio < 0.2:
                continue

            # Calculate centroid
            M = cv2.moments(smooth_contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
            else: 
                center_x, center_y = x + w // 2, y + h // 2

            # Store object information
            detected_objects.append({
                'contour': smooth_contour,
                'area': area,
                'center': (center_x, center_y),
                'bounding_rect': (x, y, w, h),
                'color': color_name,
                'aspect_ratio': aspect_ratio
            })
            
        return detected_objects
    
    def draw_detections(self, image, objects):
        """Draw detection results on the image"""
        result_image = image.copy()
        
        for obj in objects:
            # Choose color for drawing
            if obj['color'] == 'red':
                draw_color = (0, 0, 255)  # BGR format
            elif obj['color'] == 'blue':
                draw_color = (255, 0, 0)  # BGR format
            else:
                draw_color = (0, 255, 0)  # Default green
            
            # Draw bounding rectangle
            x, y, w, h = obj['bounding_rect']
            cv2.rectangle(result_image, (x, y), (x + w, y + h), draw_color, 2)
            
            # Draw center point
            center_x, center_y = obj['center']
            cv2.circle(result_image, (center_x, center_y), 5, draw_color, -1)
            
            # Add text label with additional info
            text = f"{obj['color'].upper()} ({int(obj['area'])})"
            cv2.putText(result_image, text, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, draw_color, 2)
            
            # Draw contour (optional - uncomment if needed)
            # cv2.drawContours(result_image, [obj['contour']], -1, draw_color, 2)
            
        return result_image
    
    def process_frame(self, frame, hsv_values):
        """Main processing function for each frame"""
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create masks for each color
        red_mask = self.create_mask(hsv, 'red', hsv_values)
        blue_mask = self.create_mask(hsv, 'blue', hsv_values)
        
        # Clean the masks
        red_mask_clean = self.clean_mask(red_mask)
        blue_mask_clean = self.clean_mask(blue_mask)
        
        # Detect objects
        red_objects = self.detect_objects(red_mask_clean, 'red')
        blue_objects = self.detect_objects(blue_mask_clean, 'blue')
        
        # Combine all objects
        all_objects = red_objects + blue_objects
        
        # Draw results on the original frame
        result_image = self.draw_detections(frame, all_objects)

        return result_image, red_mask_clean, blue_mask_clean, all_objects
    
    # Keep the old method name for backward compatibility
    def show(self, image, objects):
        """Backward compatibility method"""
        return self.draw_detections(image, objects)

def main():
    """Test function to run the detector standalone"""
    config = HSVConfig()  
    detector = tarp()
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return

    print("Press 'q' to quit")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Cannot read frame")
                break
            
            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            
            # Get current HSV values from trackbars
            hsv_values = config.get_hsv()
            
            # Process the frame
            result, red_mask, blue_mask, objects = detector.process_frame(frame, hsv_values)
            
            # Display results
            cv2.imshow("Terpal Detection", result)
            cv2.imshow("Red Mask", red_mask)
            cv2.imshow("Blue Mask", blue_mask)
            
            # Check for quit command
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
                
            # Print detection info
            if objects:
                for obj in objects:
                    print(f"Detected {obj['color']} object - Area: {obj['area']}, Center: {obj['center']}")
                    
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        cap.release()
        config.destroy()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()