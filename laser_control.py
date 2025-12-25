"""
Laser Control Module
Handles laser module ON/OFF control for aiming assistance.
"""

import time

try:
    import RPi.GPIO as GPIO
    RPI_AVAILABLE = True
except ImportError:
    RPI_AVAILABLE = False
    print("RPi.GPIO not available. Running in simulation mode.")


class LaserController:
    def __init__(self, laser_pin=21, use_arduino=False, servo_controller=None):
        """
        Initialize laser controller.
        
        Args:
            laser_pin (int): GPIO pin for laser module control
            use_arduino (bool): Use Arduino for laser control via servo controller
            servo_controller (ServoController): Reference to servo controller for Arduino communication
        """
        self.laser_pin = laser_pin
        self.use_arduino = use_arduino
        self.servo_controller = servo_controller
        self.laser_state = False
        
        if not self.use_arduino and RPI_AVAILABLE:
            self._init_gpio()
        elif self.use_arduino and servo_controller is None:
            print("Warning: Arduino mode enabled but no servo controller provided")
        
        print("Laser controller initialized")
    
    def _init_gpio(self):
        """Initialize GPIO for laser control."""
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.laser_pin, GPIO.OUT)
        GPIO.output(self.laser_pin, GPIO.LOW)  # Start with laser OFF
        print(f"Laser GPIO initialized on pin {self.laser_pin}")
    
    def turn_on(self):
        """Turn ON the laser module."""
        if self.laser_state:
            return  # Already on
        
        if self.use_arduino and self.servo_controller and hasattr(self.servo_controller, 'arduino'):
            command = "LASER:ON\n"
            self.servo_controller.arduino.write(command.encode())
        elif RPI_AVAILABLE:
            GPIO.output(self.laser_pin, GPIO.HIGH)
        else:
            print("Simulation: Laser ON")
        
        self.laser_state = True
        print("Laser turned ON")
    
    def turn_off(self):
        """Turn OFF the laser module."""
        if not self.laser_state:
            return  # Already off
        
        if self.use_arduino and self.servo_controller and hasattr(self.servo_controller, 'arduino'):
            command = "LASER:OFF\n"
            self.servo_controller.arduino.write(command.encode())
        elif RPI_AVAILABLE:
            GPIO.output(self.laser_pin, GPIO.LOW)
        else:
            print("Simulation: Laser OFF")
        
        self.laser_state = False
        print("Laser turned OFF")
    
    def toggle(self):
        """Toggle laser state."""
        if self.laser_state:
            self.turn_off()
        else:
            self.turn_on()
    
    def is_on(self):
        """
        Check if laser is currently on.
        
        Returns:
            bool: True if laser is on
        """
        return self.laser_state
    
    def pulse(self, duration=0.1, pulses=3, interval=0.1):
        """
        Pulse the laser for signaling.
        
        Args:
            duration (float): Duration of each pulse in seconds
            pulses (int): Number of pulses
            interval (float): Interval between pulses in seconds
        """
        original_state = self.laser_state
        
        for i in range(pulses):
            self.turn_on()
            time.sleep(duration)
            self.turn_off()
            if i < pulses - 1:  # Don't wait after last pulse
                time.sleep(interval)
        
        # Restore original state
        if original_state:
            self.turn_on()
    
    def cleanup(self):
        """Clean up GPIO resources."""
        self.turn_off()  # Ensure laser is off
        
        if not self.use_arduino and RPI_AVAILABLE:
            GPIO.cleanup(self.laser_pin)
        
        print("Laser controller cleaned up")