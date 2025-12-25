"""
Servo Control Module
Handles pan/tilt servo motors and trigger mechanism for balloon targeting system.
"""

import time
import math

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False
    print("RPi.GPIO not available. Running in simulation mode.")

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("pyserial not available. Arduino communication disabled.")


class ServoController:
    def __init__(self, pan_pin=18, tilt_pin=19, trigger_pin=20, 
                 use_arduino=False, arduino_port='/dev/ttyUSB0', arduino_baudrate=9600):
        """
        Initialize servo controller.
        
        Args:
            pan_pin (int): GPIO pin for pan servo (horizontal movement)
            tilt_pin (int): GPIO pin for tilt servo (vertical movement)
            trigger_pin (int): GPIO pin for trigger servo
            use_arduino (bool): Use Arduino for servo control via serial
            arduino_port (str): Arduino serial port
            arduino_baudrate (int): Arduino serial baudrate
        """
        self.pan_pin = pan_pin
        self.tilt_pin = tilt_pin
        self.trigger_pin = trigger_pin
        self.use_arduino = use_arduino
        
        # Servo angle limits and current positions
        self.pan_angle = 90  # Center position
        self.tilt_angle = 90  # Center position
        self.pan_min, self.pan_max = 0, 180
        self.tilt_min, self.tilt_max = 45, 135  # Limited vertical range for safety
        
        # PWM frequency and duty cycle mapping
        self.pwm_frequency = 50  # 50Hz for standard servos
        
        if self.use_arduino and SERIAL_AVAILABLE:
            self._init_arduino(arduino_port, arduino_baudrate)
        elif RPI_AVAILABLE:
            self._init_gpio()
        else:
            print("Running in simulation mode - no hardware control")
    
    def _init_gpio(self):
        """Initialize Raspberry Pi GPIO for servo control."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pan_pin, GPIO.OUT)
        GPIO.setup(self.tilt_pin, GPIO.OUT)
        GPIO.setup(self.trigger_pin, GPIO.OUT)
        
        # Initialize PWM
        self.pan_pwm = GPIO.PWM(self.pan_pin, self.pwm_frequency)
        self.tilt_pwm = GPIO.PWM(self.tilt_pin, self.pwm_frequency)
        self.trigger_pwm = GPIO.PWM(self.trigger_pin, self.pwm_frequency)
        
        # Start PWM with center positions
        self.pan_pwm.start(self._angle_to_duty_cycle(90))
        self.tilt_pwm.start(self._angle_to_duty_cycle(90))
        self.trigger_pwm.start(self._angle_to_duty_cycle(0))  # Trigger at rest position
        
        print("GPIO initialized for servo control")
    
    def _init_arduino(self, port, baudrate):
        """Initialize Arduino serial communication."""
        try:
            self.arduino = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)  # Wait for Arduino to initialize
            print(f"Arduino connected on {port}")
        except Exception as e:
            print(f"Failed to connect to Arduino: {e}")
            self.use_arduino = False
    
    def _angle_to_duty_cycle(self, angle):
        """
        Convert servo angle to PWM duty cycle.
        
        Args:
            angle (float): Servo angle (0-180 degrees)
            
        Returns:
            float: PWM duty cycle percentage
        """
        # Standard servo: 1ms (5% duty) = 0°, 2ms (10% duty) = 180°
        duty_cycle = 2.5 + (angle / 180.0) * 10.0
        return duty_cycle
    
    def move_to_position(self, pan_angle, tilt_angle):
        """
        Move servos to specified angles.
        
        Args:
            pan_angle (float): Pan servo angle (0-180)
            tilt_angle (float): Tilt servo angle (45-135)
        """
        # Clamp angles to safe limits
        pan_angle = max(self.pan_min, min(self.pan_max, pan_angle))
        tilt_angle = max(self.tilt_min, min(self.tilt_max, tilt_angle))
        
        self.pan_angle = pan_angle
        self.tilt_angle = tilt_angle
        
        if self.use_arduino and hasattr(self, 'arduino'):
            self._send_arduino_command(pan_angle, tilt_angle)
        elif RPI_AVAILABLE:
            self._move_gpio_servos(pan_angle, tilt_angle)
        else:
            print(f"Simulation: Moving to Pan={pan_angle:.1f}°, Tilt={tilt_angle:.1f}°")
    
    def _move_gpio_servos(self, pan_angle, tilt_angle):
        """Move servos using GPIO PWM."""
        pan_duty = self._angle_to_duty_cycle(pan_angle)
        tilt_duty = self._angle_to_duty_cycle(tilt_angle)
        
        self.pan_pwm.ChangeDutyCycle(pan_duty)
        self.tilt_pwm.ChangeDutyCycle(tilt_duty)
        
        # Small delay for servo movement
        time.sleep(0.1)
    
    def _send_arduino_command(self, pan_angle, tilt_angle):
        """Send servo commands to Arduino via serial."""
        command = f"PAN:{pan_angle:.1f},TILT:{tilt_angle:.1f}\n"
        self.arduino.write(command.encode())
    
    def calculate_target_angles(self, target_x, target_y, frame_center_x, frame_center_y, 
                              frame_width, frame_height, fov_horizontal=60, fov_vertical=45):
        """
        Calculate servo angles needed to aim at target coordinates.
        
        Args:
            target_x, target_y (int): Target pixel coordinates
            frame_center_x, frame_center_y (int): Frame center coordinates
            frame_width, frame_height (int): Frame dimensions
            fov_horizontal, fov_vertical (float): Camera field of view in degrees
            
        Returns:
            tuple: (pan_angle, tilt_angle)
        """
        # Calculate pixel offset from center
        offset_x = target_x - frame_center_x
        offset_y = target_y - frame_center_y
        
        # Convert pixel offset to angle offset
        pixels_per_degree_x = frame_width / fov_horizontal
        pixels_per_degree_y = frame_height / fov_vertical
        
        angle_offset_x = offset_x / pixels_per_degree_x
        angle_offset_y = offset_y / pixels_per_degree_y
        
        # Calculate new servo angles (inverted for proper movement direction)
        new_pan_angle = self.pan_angle - angle_offset_x
        new_tilt_angle = self.tilt_angle + angle_offset_y  # Inverted Y-axis
        
        return new_pan_angle, new_tilt_angle
    
    def is_target_aligned(self, target_x, target_y, frame_center_x, frame_center_y, 
                         tolerance=20):
        """
        Check if target is aligned within tolerance.
        
        Args:
            target_x, target_y (int): Target coordinates
            frame_center_x, frame_center_y (int): Frame center coordinates
            tolerance (int): Alignment tolerance in pixels
            
        Returns:
            bool: True if target is aligned
        """
        distance = math.sqrt((target_x - frame_center_x)**2 + (target_y - frame_center_y)**2)
        return distance <= tolerance
    
    def fire_trigger(self, fire_duration=0.5):
        """
        Activate trigger servo to fire.
        
        Args:
            fire_duration (float): Duration to hold trigger in seconds
        """
        if self.use_arduino and hasattr(self, 'arduino'):
            command = f"FIRE:{fire_duration}\n"
            self.arduino.write(command.encode())
        elif RPI_AVAILABLE:
            # Move trigger servo to fire position
            fire_duty = self._angle_to_duty_cycle(90)  # Adjust based on trigger mechanism
            self.trigger_pwm.ChangeDutyCycle(fire_duty)
            time.sleep(fire_duration)
            
            # Return to rest position
            rest_duty = self._angle_to_duty_cycle(0)
            self.trigger_pwm.ChangeDutyCycle(rest_duty)
        else:
            print(f"Simulation: FIRING for {fire_duration} seconds!")
        
        time.sleep(fire_duration)
    
    def center_servos(self):
        """Move servos to center position."""
        self.move_to_position(90, 90)
    
    def cleanup(self):
        """Clean up GPIO resources."""
        if RPI_AVAILABLE and hasattr(self, 'pan_pwm'):
            self.pan_pwm.stop()
            self.tilt_pwm.stop()
            self.trigger_pwm.stop()
            GPIO.cleanup()
        
        if self.use_arduino and hasattr(self, 'arduino'):
            self.arduino.close()
        
        print("Servo controller cleaned up")