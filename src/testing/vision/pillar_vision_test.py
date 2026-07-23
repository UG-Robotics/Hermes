import time
import sys
from picamera2 import Picamera2

# Import project specific configuration layers
from config.camera_config import (
    FRAME_WIDTH, FRAME_HEIGHT,
    PILLAR_LATERAL_OFFSET_PX, PILLAR_STEER_KP,
)
from config.robot_config import SPEED_DEFAULT_FORWARD, STEER_MIN, STEER_MAX

# Import the vision engine from your preprocessing script
from perception.preprocessing import detect_pillars

def run_standalone_test():
    print("\n=======================================================")
    print("WRO 2026 - Standalone Vision Control Loop Tester Active")
    print("=======================================================")
    
    # 1. Initialize the Pi Camera Module V2 hardware array via Picamera2
    try:
        camera = Picamera2()
        camera.configure(camera.create_video_configuration(main={"size": (FRAME_WIDTH, FRAME_HEIGHT)}))
        camera.start()
        print("SUCCESS: Pi Camera V2 initialized and streaming raw RGB arrays.")
    except Exception as e:
        print(f"HARDWARE ERROR: Failed to interface with Pi Cam V2 via libcamera: {e}")
        return

    # 2. Rule Compliant Terminal Bypass Waiting State
    print("\n=======================================================")
    print("WRO STATUS: IDLE. Camera is armed and standing still.")
    print("Press [ENTER] inside this SSH terminal session to launch loop...")
    print("=======================================================")
    input()
    print("WRO STATUS: MATCH STARTED. Tracking execution loop running at 20 Hz...\n")

    loop_interval = 1.0 / 20.0  # 20 Hz (0.05 seconds per cycle)
    
    try:
        while True:
            start_time = time.time()
            
            # Capture frame natively as an optimized RGB NumPy matrix
            frame_rgb = camera.capture_array()
            
            # Extract features and calculate center mass coordinates via your preprocessing file
            pillar_type, cx = detect_pillars(frame_rgb)
            
            # Default tracking defaults (when no objects are visible)
            speed = SPEED_DEFAULT_FORWARD
            steer = 0
            
            # If an obstacle is detected, calculate proportional steering trajectory shift
            if pillar_type is not None and cx is not None:
                screen_center = FRAME_WIDTH // 2
                
                # Use the SAME aim-point offset + gain the runtime pillar
                # planner uses (config/camera_config.py) so this bench demo
                # can't drift from perception/pillar_detection.compute_steer_angle.
                if pillar_type == 'RED':
                    target_x = screen_center + PILLAR_LATERAL_OFFSET_PX  # pass RIGHT of red
                else:
                    target_x = screen_center - PILLAR_LATERAL_OFFSET_PX  # pass LEFT of green

                error = target_x - cx
                steer = int(error * PILLAR_STEER_KP)
                
                # Protect mechanical limits by clamping the variable
                steer = max(STEER_MIN, min(steer, STEER_MAX))
            
            # Output live tracking diagnostic telemetry straight to console
            print(f"TRACKING: {str(pillar_type):<5} | CENTROID X: {str(cx):<4} | "
                  f"TARGET SPEED: {speed} | CALCULATED STEER: {steer:+3} deg")
            
            # Enforce strict loop execution interval timing
            elapsed = time.time() - start_time
            time.sleep(max(0.0, loop_interval - elapsed))
            
    except KeyboardInterrupt:
        print("\nExecution terminated manually via terminal interrupt.")
    finally:
        # Guarantee physical device teardown to clear device busy hardware locks
        print("Cleaning up camera driver links...")
        camera.stop()
        camera.close()
        print("Standalone verification completed successfully.")


if __name__ == '__main__':
    run_standalone_test()