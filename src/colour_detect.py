import cv2
import numpy as np
from hsv_config import HSVConfig  

class tarp:
    def __init__(self):
        # Inisialisasi kernel dan nilai tetap lainnya
        self.kernel_small = np.ones((3, 3), np.uint8)
        self.kernel_medium = np.ones((5, 5), np.uint8)
        self.kernel_large = np.ones((7, 7), np.uint8)
        self.kernel_ellipse = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        self.min_area = 500
        
    def create_mask(self, hsv_image, color_type, hsv_values): 
        if color_type == 'red':
            mask1 = cv2.inRange(hsv_image, hsv_values['red_low1'], hsv_values['red_up1'])
            mask2 = cv2.inRange(hsv_image, hsv_values['red_low2'], hsv_values['red_up2'])
            mask = cv2.bitwise_or(mask1, mask2)
        elif color_type == 'blue':
            mask = cv2.inRange(hsv_image, hsv_values['blue_low'], hsv_values['blue_up'])
        else:
            return None
        return mask

    def clean_mask(self, mask):
        mask = cv2.medianBlur(mask, 5)
        opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_small, iterations=2)
        closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, self.kernel_medium, iterations=2)
        smooth = cv2.morphologyEx(closing, cv2.MORPH_OPEN, self.kernel_ellipse, iterations=1)
        final = cv2.morphologyEx(smooth, cv2.MORPH_CLOSE, self.kernel_large, iterations=1)
        final = cv2.GaussianBlur(final, (3, 3), 0)
        _, final = cv2.threshold(final, 127, 255, cv2.THRESH_BINARY)
        return final
    
    def detect_objects(self, mask, color_name):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        obj_detect = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_area:
                continue
            
            epsilon = 0.02 * cv2.arcLength(contour, True)
            smooth_contour = cv2.approxPolyDP(contour, epsilon, True)
            x, y, w, h = cv2.boundingRect(smooth_contour)
            aspect_ratio = w / h
            if aspect_ratio > 5 or aspect_ratio < 0.2:
                continue

            M = cv2.moments(smooth_contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
            else: 
                center_x, center_y = x + w // 2, y + h // 2

            obj_detect.append({
                'contour': smooth_contour, 'area': area, 'center': (center_x, center_y),
                'bounding_rect': (x, y, w, h), 'color': color_name, 'aspect_ratio': aspect_ratio
            })
        return obj_detect
    
    def show(self, image, objects):
        result_image = image.copy()
        for obj in objects:
            color = (0, 0, 255) if obj['color'] == 'red' else (255, 0, 0)
            x, y, w, h = obj['bounding_rect']
            cv2.rectangle(result_image, (x, y), (x + w, y + h), color, 2)
            center_x, center_y = obj['center']
            cv2.circle(result_image, (center_x, center_y), 5, color, -1)
            text = obj['color'].upper()
            cv2.putText(result_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        return result_image
    
    def process_frame(self, frame, hsv_values):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        red_mask = self.create_mask(hsv, 'red', hsv_values)
        blue_mask = self.create_mask(hsv, 'blue', hsv_values)
        
        red_mask_clean = self.clean_mask(red_mask)
        blue_mask_clean = self.clean_mask(blue_mask)
        
        red_objects = self.detect_objects(red_mask_clean, 'red')
        blue_objects = self.detect_objects(blue_mask_clean, 'blue')
        
        all_objects = red_objects + blue_objects
        result_image = self.show(frame, all_objects)

        return result_image, red_mask_clean, blue_mask_clean, all_objects

def main():
    config = HSVConfig()  
    detector = tarp()
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Tidak bisa membuka kamera")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: Tidak bisa membaca frame")
            break
        
        frame = cv2.flip(frame, 1)
        
        # ambil dan kirim nilai HSV terbaru di setiap frame
        hsv_values = config.get_hsv()
        result, red_mask, blue_mask, objects = detector.process_frame(frame, hsv_values)
        
        cv2.imshow("Terpal", result)
        cv2.imshow("Mask Merah", red_mask)
        cv2.imshow("Mask Biru", blue_mask)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()