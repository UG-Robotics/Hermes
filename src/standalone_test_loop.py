import time
import sys
import pathlib

# Ensure we can resolve imports relative to the 'src' directory
current_file = pathlib.Path(__file__).resolve()
src_dir = current_file.parent
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

from picamera2 import Picamera2
from perception.preprocessing import detect_pillars

def main():
    # Initialize and configure Picamera2
    print("Initializing Picamera2...")
    camera = Picamera2()
    
    # Configure main stream to 640x480 resolution with RGB888 format
    # In Picamera2, 'RGB888' format yields standard BGR ordered NumPy arrays
    config = camera.create_preview_configuration(main={"format": "RGB888", "size": (640, 480)})
    camera.configure(config)
    
    # Start the camera stream
    camera.start()
    print("Camera stream booted successfully.")
    
    # Blocking terminal start trigger to satisfy autonomous rules without a push button
    input("System Idle. Press [ENTER] in the terminal to start the autonomous control loop...")
    print("Autonomous tracking loop started.")
    
    kp = 0.5
    loop_interval = 0.05  # 20 Hz loop frequency (0.05 seconds interval)
    
    try:
        while True:
            start_time = time.time()
            
            # Capture frame natively via Picamera2 (returns NumPy array)
            frame = camera.capture_array()
            
            # Extract dominant color and centroid X position
            pillar_type, cx = detect_pillars(frame)
            
            # Proportional controller and target calculation
            if pillar_type is not None and cx is not None:
                if pillar_type == 'RED':
                    # Rule: Red pillar must be passed on the right -> offset target to the right side
                    target_x = 320 + 180  # 500
                elif pillar_type == 'GREEN':
                    # Rule: Green pillar must be passed on the left -> offset target to the left side
                    target_x = 320 - 180  # 140
                else:
                    target_x = 320
                
                # Compute proportional steering vector error scaled by kp
                error = target_x - cx
                steer = error * kp
                
                # Enforce steering clamp limits [-90, 90]
                steer = max(-90.0, min(90.0, steer))
            else:
                steer = 0.0
            
            # Print diagnostics on every cycle
            print(f"TARGET: {pillar_type} | CX: {cx} | STEER OUT: {steer}")
            sys.stdout.flush()
            
            # Keep loop timing strictly at 20Hz
            elapsed = time.time() - start_time
            sleep_time = loop_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
                
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt detected. Shutting down...")
    finally:
        # Guarantee clean execution of camera.stop() and camera.close()
        print("Stopping and closing camera array...")
        try:
            camera.stop()
        except Exception as e:
            print(f"Error stopping camera: {e}", file=sys.stderr)
        try:
            camera.close()
        except Exception as e:
            print(f"Error closing camera: {e}", file=sys.stderr)
        print("Cleanup completed.")

if __name__ == '__main__':
    main()
