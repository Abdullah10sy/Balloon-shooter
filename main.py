"""
Balloon Targeting System - Main Application
Integrates YOLOv5 detection, servo control, and laser aiming for automated balloon targeting.
"""

import cv2
import time
import numpy as np
import argparse
from datetime import datetime, timedelta

from detector import BalloonDetector
from servo_control import ServoController
from laser_control import LaserController


class BalloonTargetingSystem:
    def __init__(self, model_path='yolov5s.pt', camera_index=0, 
                 use_arduino=False, arduino_port='/dev/ttyUSB0'):
        """
        Initialize the balloon targeting system.
        
        Args:
            model_path (str): Path to YOLOv5 model
            camera_index (int): Camera device index
            use_arduino (bool): Use Arduino for hardware control
            arduino_port (str): Arduino serial port
        """
        print("Initializing Balloon Targeting System...")
        
        # Initialize components
        self.detector = BalloonDetector(model_path)
        self.servo_controller = ServoController(use_arduino=use_arduino, 
                                              arduino_port=arduino_port)
        self.laser_controller = LaserController(use_arduino=use_arduino, 
                                              servo_controller=self.servo_controller)
        
        # Camera setup
        self.camera = cv2.VideoCapture(camera_index)
        if not self.camera.isOpened():
            raise RuntimeError(f"Failed to open camera {camera_index}")
        
        # Set camera resolution (adjust based on your camera)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # System parameters
        self.alignment_tolerance = 20  # pixels
        self.firing_delay = 3.0  # seconds between shots
        self.last_fire_time = None
        self.tracking_enabled = True
        self.auto_fire_enabled = False
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        
        print("System initialized successfully!")
        print("Controls:")
        print("  SPACE - Toggle auto-fire mode")
        print("  'l' - Toggle laser")
        print("  'c' - Center servos")
        print("  'f' - Manual fire")
        print("  't' - Toggle tracking")
        print("  'q' - Quit")
    
    def process_frame(self, frame):
        """
        Process a single frame for balloon detection and targeting.
        
        Args:
            frame (numpy.ndarray): Input camera frame
            
        Returns:
            numpy.ndarray: Processed frame with annotations
        """
        # Detect balloons
        detections = self.detector.detect_balloons(frame)
        
        # Get frame center
        frame_center_x, frame_center_y = self.detector.get_frame_center(frame)
        
        # Draw frame center crosshair
        cv2.line(frame, (frame_center_x - 20, frame_center_y), 
                (frame_center_x + 20, frame_center_y), (255, 255, 255), 2)
        cv2.line(frame, (frame_center_x, frame_center_y - 20), 
                (frame_center_x, frame_center_y + 20), (255, 255, 255), 2)
        
        # Process detections
        target_balloon = None
        if len(detections) == 1 and self.tracking_enabled:
            # Single balloon detected - safe to target
            target_balloon = detections[0]
            target_x, target_y = self.detector.get_balloon_center(target_balloon)
            
            # Calculate servo angles for targeting
            frame_height, frame_width = frame.shape[:2]
            pan_angle, tilt_angle = self.servo_controller.calculate_target_angles(
                target_x, target_y, frame_center_x, frame_center_y,
                frame_width, frame_height
            )
            
            # Move servos to track target
            self.servo_controller.move_to_position(pan_angle, tilt_angle)
            
            # Turn on laser for aiming
            if not self.laser_controller.is_on():
                self.laser_controller.turn_on()
            
            # Check if target is aligned
            is_aligned = self.servo_controller.is_target_aligned(
                target_x, target_y, frame_center_x, frame_center_y, 
                self.alignment_tolerance
            )
            
            # Draw alignment status
            alignment_color = (0, 255, 0) if is_aligned else (0, 255, 255)
            cv2.circle(frame, (target_x, target_y), self.alignment_tolerance, 
                      alignment_color, 2)
            
            # Auto-fire logic
            if (is_aligned and self.auto_fire_enabled and 
                self._can_fire()):
                self._fire_at_target()
            
            # Draw targeting line
            cv2.line(frame, (frame_center_x, frame_center_y), 
                    (target_x, target_y), (0, 255, 255), 1)
            
        elif len(detections) > 1:
            # Multiple balloons - unsafe to fire
            self.laser_controller.turn_off()
            cv2.putText(frame, "MULTIPLE TARGETS - UNSAFE", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        elif len(detections) == 0:
            # No balloons detected
            self.laser_controller.turn_off()
        
        # Draw all detections
        frame = self.detector.draw_detections(frame, detections)
        
        # Draw system status
        self._draw_status(frame, len(detections), target_balloon is not None)
        
        return frame
    
    def _can_fire(self):
        """
        Check if system can fire based on safety conditions.
        
        Returns:
            bool: True if safe to fire
        """
        if self.last_fire_time is None:
            return True
        
        time_since_last_fire = time.time() - self.last_fire_time
        return time_since_last_fire >= self.firing_delay
    
    def _fire_at_target(self):
        """Execute firing sequence."""
        print("FIRING AT TARGET!")
        
        # Brief laser pulse to indicate firing
        self.laser_controller.pulse(duration=0.05, pulses=2, interval=0.05)
        
        # Fire trigger
        self.servo_controller.fire_trigger()
        
        # Turn off laser after firing
        self.laser_controller.turn_off()
        
        # Update last fire time
        self.last_fire_time = time.time()
    
    def _draw_status(self, frame, detection_count, is_tracking):
        """Draw system status information on frame."""
        y_offset = 60
        
        # Detection count
        cv2.putText(frame, f"Balloons: {detection_count}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
        
        # Tracking status
        track_color = (0, 255, 0) if is_tracking else (0, 0, 255)
        track_text = "TRACKING" if is_tracking else "NOT TRACKING"
        cv2.putText(frame, f"Status: {track_text}", (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, track_color, 2)
        y_offset += 25
        
        # Auto-fire status
        fire_color = (0, 255, 0) if self.auto_fire_enabled else (0, 0, 255)
        fire_text = "AUTO-FIRE ON" if self.auto_fire_enabled else "MANUAL FIRE"
        cv2.putText(frame, fire_text, (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, fire_color, 2)
        y_offset += 25
        
        # Laser status
        laser_color = (0, 255, 0) if self.laser_controller.is_on() else (128, 128, 128)
        laser_text = "LASER ON" if self.laser_controller.is_on() else "LASER OFF"
        cv2.putText(frame, laser_text, (10, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, laser_color, 2)
        y_offset += 25
        
        # Servo positions
        cv2.putText(frame, f"Pan: {self.servo_controller.pan_angle:.1f}°", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        y_offset += 20
        cv2.putText(frame, f"Tilt: {self.servo_controller.tilt_angle:.1f}°", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # FPS counter
        self.frame_count += 1
        elapsed_time = time.time() - self.start_time
        if elapsed_time > 0:
            fps = self.frame_count / elapsed_time
            cv2.putText(frame, f"FPS: {fps:.1f}", (frame.shape[1] - 100, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    
    def run(self):
        """Main system loop."""
        print("Starting balloon targeting system...")
        print("Press 'q' to quit")
        
        # Center servos at startup
        self.servo_controller.center_servos()
        
        try:
            while True:
                # Capture frame
                ret, frame = self.camera.read()
                if not ret:
                    print("Failed to capture frame")
                    break
                
                # Process frame
                processed_frame = self.process_frame(frame)
                
                # Display frame
                cv2.imshow('Balloon Targeting System', processed_frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord(' '):  # Space bar
                    self.auto_fire_enabled = not self.auto_fire_enabled
                    print(f"Auto-fire: {'ENABLED' if self.auto_fire_enabled else 'DISABLED'}")
                elif key == ord('l'):
                    self.laser_controller.toggle()
                elif key == ord('c'):
                    self.servo_controller.center_servos()
                    print("Servos centered")
                elif key == ord('f'):
                    if self._can_fire():
                        self._fire_at_target()
                    else:
                        print("Cannot fire - cooling down")
                elif key == ord('t'):
                    self.tracking_enabled = not self.tracking_enabled
                    print(f"Tracking: {'ENABLED' if self.tracking_enabled else 'DISABLED'}")
        
        except KeyboardInterrupt:
            print("\nShutdown requested...")
        
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up system resources."""
        print("Cleaning up system...")
        
        # Turn off laser
        self.laser_controller.turn_off()
        
        # Center servos
        self.servo_controller.center_servos()
        
        # Release camera
        self.camera.release()
        cv2.destroyAllWindows()
        
        # Clean up controllers
        self.servo_controller.cleanup()
        self.laser_controller.cleanup()
        
        print("System shutdown complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='Balloon Targeting System')
    parser.add_argument('--model', default='yolov5s.pt', 
                       help='Path to YOLOv5 model file')
    parser.add_argument('--camera', type=int, default=0, 
                       help='Camera device index')
    parser.add_argument('--arduino', action='store_true', 
                       help='Use Arduino for hardware control')
    parser.add_argument('--port', default='/dev/ttyUSB0', 
                       help='Arduino serial port')
    
    args = parser.parse_args()
    
    try:
        system = BalloonTargetingSystem(
            model_path=args.model,
            camera_index=args.camera,
            use_arduino=args.arduino,
            arduino_port=args.port
        )
        system.run()
    
    except Exception as e:
        print(f"Error: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())