import cv2
import numpy as np

class HSVConfig:
    def __init__(self):
        """Initialize HSV configuration with trackbars for real-time tuning"""
        # Create window for trackbars
        cv2.namedWindow('HSV Controls', cv2.WINDOW_AUTOSIZE)
        
        # Default HSV values for red color (two ranges because red wraps around 0° in HSV)
        # Red range 1 (lower reds: 0-10°)
        cv2.createTrackbar('Red1 H Min', 'HSV Controls', 0, 179, self.nothing)
        cv2.createTrackbar('Red1 S Min', 'HSV Controls', 50, 255, self.nothing)
        cv2.createTrackbar('Red1 V Min', 'HSV Controls', 50, 255, self.nothing)
        cv2.createTrackbar('Red1 H Max', 'HSV Controls', 10, 179, self.nothing)
        cv2.createTrackbar('Red1 S Max', 'HSV Controls', 255, 255, self.nothing)
        cv2.createTrackbar('Red1 V Max', 'HSV Controls', 255, 255, self.nothing)
        
        # Red range 2 (higher reds: 170-179°)
        cv2.createTrackbar('Red2 H Min', 'HSV Controls', 170, 179, self.nothing)
        cv2.createTrackbar('Red2 S Min', 'HSV Controls', 50, 255, self.nothing)
        cv2.createTrackbar('Red2 V Min', 'HSV Controls', 50, 255, self.nothing)
        cv2.createTrackbar('Red2 H Max', 'HSV Controls', 179, 179, self.nothing)
        cv2.createTrackbar('Red2 S Max', 'HSV Controls', 255, 255, self.nothing)
        cv2.createTrackbar('Red2 V Max', 'HSV Controls', 255, 255, self.nothing)
        
        # Blue range
        cv2.createTrackbar('Blue H Min', 'HSV Controls', 100, 179, self.nothing)
        cv2.createTrackbar('Blue S Min', 'HSV Controls', 50, 255, self.nothing)
        cv2.createTrackbar('Blue V Min', 'HSV Controls', 50, 255, self.nothing)
        cv2.createTrackbar('Blue H Max', 'HSV Controls', 130, 179, self.nothing)
        cv2.createTrackbar('Blue S Max', 'HSV Controls', 255, 255, self.nothing)
        cv2.createTrackbar('Blue V Max', 'HSV Controls', 255, 255, self.nothing)
        
        print("HSV Configuration initialized. Use trackbars to adjust color ranges.")
        print("Red spans two ranges due to HSV color wheel wraparound.")
    
    def nothing(self, val):
        """Callback function for trackbars (does nothing)"""
        pass
    
    def get_hsv(self):
        """Get current HSV values from trackbars"""
        hsv_values = {
            # Red range 1 (0-10°)
            'red_low1': np.array([
                cv2.getTrackbarPos('Red1 H Min', 'HSV Controls'),
                cv2.getTrackbarPos('Red1 S Min', 'HSV Controls'),
                cv2.getTrackbarPos('Red1 V Min', 'HSV Controls')
            ]),
            'red_up1': np.array([
                cv2.getTrackbarPos('Red1 H Max', 'HSV Controls'),
                cv2.getTrackbarPos('Red1 S Max', 'HSV Controls'),
                cv2.getTrackbarPos('Red1 V Max', 'HSV Controls')
            ]),
            
            # Red range 2 (170-179°)
            'red_low2': np.array([
                cv2.getTrackbarPos('Red2 H Min', 'HSV Controls'),
                cv2.getTrackbarPos('Red2 S Min', 'HSV Controls'),
                cv2.getTrackbarPos('Red2 V Min', 'HSV Controls')
            ]),
            'red_up2': np.array([
                cv2.getTrackbarPos('Red2 H Max', 'HSV Controls'),
                cv2.getTrackbarPos('Red2 S Max', 'HSV Controls'),
                cv2.getTrackbarPos('Red2 V Max', 'HSV Controls')
            ]),
            
            # Blue range
            'blue_low': np.array([
                cv2.getTrackbarPos('Blue H Min', 'HSV Controls'),
                cv2.getTrackbarPos('Blue S Min', 'HSV Controls'),
                cv2.getTrackbarPos('Blue V Min', 'HSV Controls')
            ]),
            'blue_up': np.array([
                cv2.getTrackbarPos('Blue H Max', 'HSV Controls'),
                cv2.getTrackbarPos('Blue S Max', 'HSV Controls'),
                cv2.getTrackbarPos('Blue V Max', 'HSV Controls')
            ])
        }
        return hsv_values
    
    def get_preset_values(self, preset="default"):
        """Get preset HSV values without trackbars (useful for autonomous operation)"""
        if preset == "default":
            return {
                'red_low1': np.array([0, 50, 50]),
                'red_up1': np.array([10, 255, 255]),
                'red_low2': np.array([170, 50, 50]),
                'red_up2': np.array([179, 255, 255]),
                'blue_low': np.array([100, 50, 50]),
                'blue_up': np.array([130, 255, 255])
            }
        elif preset == "bright":
            return {
                'red_low1': np.array([0, 70, 70]),
                'red_up1': np.array([10, 255, 255]),
                'red_low2': np.array([170, 70, 70]),
                'red_up2': np.array([179, 255, 255]),
                'blue_low': np.array([100, 70, 70]),
                'blue_up': np.array([130, 255, 255])
            }
        elif preset == "low_light":
            return {
                'red_low1': np.array([0, 30, 30]),
                'red_up1': np.array([15, 255, 255]),
                'red_low2': np.array([165, 30, 30]),
                'red_up2': np.array([179, 255, 255]),
                'blue_low': np.array([95, 30, 30]),
                'blue_up': np.array([135, 255, 255])
            }
        else:
            return self.get_preset_values("default")
    
    def save_config(self, filename="hsv_config.txt"):
        """Save current trackbar values to file"""
        hsv_values = self.get_hsv()
        try:
            with open(filename, 'w') as f:
                for key, value in hsv_values.items():
                    f.write(f"{key}: {value[0]}, {value[1]}, {value[2]}\n")
            print(f"HSV configuration saved to {filename}")
        except Exception as e:
            print(f"Error saving config: {e}")
    
    def load_config(self, filename="hsv_config.txt"):
        """Load HSV values from file and set trackbars"""
        try:
            config = {}
            with open(filename, 'r') as f:
                for line in f:
                    if ':' in line:
                        key, values = line.strip().split(':')
                        values = [int(x.strip()) for x in values.split(',')]
                        config[key.strip()] = values
            
            # Set trackbar positions
            if 'red_low1' in config:
                cv2.setTrackbarPos('Red1 H Min', 'HSV Controls', config['red_low1'][0])
                cv2.setTrackbarPos('Red1 S Min', 'HSV Controls', config['red_low1'][1])
                cv2.setTrackbarPos('Red1 V Min', 'HSV Controls', config['red_low1'][2])
            
            if 'red_up1' in config:
                cv2.setTrackbarPos('Red1 H Max', 'HSV Controls', config['red_up1'][0])
                cv2.setTrackbarPos('Red1 S Max', 'HSV Controls', config['red_up1'][1])
                cv2.setTrackbarPos('Red1 V Max', 'HSV Controls', config['red_up1'][2])
                
            # Continue for other ranges...
            print(f"HSV configuration loaded from {filename}")
            
        except Exception as e:
            print(f"Error loading config: {e}")
            print("Using default values")
    
    def destroy(self):
        """Clean up OpenCV windows"""
        cv2.destroyWindow('HSV Controls')

def main():
    """Test function for HSV configuration"""
    config = HSVConfig()
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("Error: Cannot open camera")
        return
    
    print("Press 'q' to quit, 's' to save config, 'l' to load config")
    
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Flip frame horizontally for mirror effect
            frame = cv2.flip(frame, 1)
            
            # Convert to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Get current HSV values
            hsv_values = config.get_hsv()
            
            # Create masks
            red_mask1 = cv2.inRange(hsv, hsv_values['red_low1'], hsv_values['red_up1'])
            red_mask2 = cv2.inRange(hsv, hsv_values['red_low2'], hsv_values['red_up2'])
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            
            blue_mask = cv2.inRange(hsv, hsv_values['blue_low'], hsv_values['blue_up'])
            
            # Show results
            cv2.imshow('Original', frame)
            cv2.imshow('Red Mask', red_mask)
            cv2.imshow('Blue Mask', blue_mask)
            
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                config.save_config()
            elif key == ord('l'):
                config.load_config()
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        cap.release()
        config.destroy()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
