# Balloon Targeting System

An automated balloon detection and targeting system using YOLOv5 computer vision, servo-controlled aiming, and laser guidance.

## Features

- **Real-time balloon detection** using YOLOv5 neural network
- **Automated servo aiming** with pan/tilt control
- **Laser aiming assistance** for precise targeting
- **Safety mechanisms** to prevent accidental firing
- **Multiple control modes** (manual/automatic)
- **Hardware flexibility** (Raspberry Pi GPIO or Arduino control)

## System Requirements

### Hardware
- USB or Raspberry Pi camera
- 2x servo motors (pan/tilt mechanism)
- 1x servo motor (trigger mechanism)
- Laser module with relay/transistor control
- Raspberry Pi 4 or Arduino Uno/Nano
- Appropriate mounting hardware

### Software
- Python 3.8+
- PyTorch
- OpenCV
- YOLOv5 (automatically downloaded)

## Installation

1. **Clone or download the project files**

2. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **For Raspberry Pi GPIO control:**
   ```bash
   pip install RPi.GPIO
   ```

4. **For Arduino control:**
   ```bash
   pip install pyserial
   ```
   Upload `arduino_code.ino` to your Arduino board.

## Hardware Setup

### Raspberry Pi GPIO Connections
```
Pan Servo    -> GPIO 18 (PWM)
Tilt Servo   -> GPIO 19 (PWM)
Trigger Servo-> GPIO 20 (PWM)
Laser Module -> GPIO 21 (Digital Out)
```

### Arduino Connections
```
Pan Servo    -> Pin 9 (PWM)
Tilt Servo   -> Pin 10 (PWM)
Trigger Servo-> Pin 11 (PWM)
Laser Module -> Pin 12 (Digital Out)
```

## Usage

### Basic Usage (Raspberry Pi)
```bash
python main.py
```

### With Custom Model
```bash
python main.py --model path/to/your/balloon_model.pt
```

### Arduino Control
```bash
python main.py --arduino --port /dev/ttyUSB0
```

### Windows Arduino Control
```bash
python main.py --arduino --port COM3
```

## Controls

| Key | Action |
|-----|--------|
| `SPACE` | Toggle auto-fire mode |
| `l` | Toggle laser on/off |
| `c` | Center servos |
| `f` | Manual fire |
| `t` | Toggle tracking |
| `q` | Quit program |

## Safety Features

1. **Single Target Only**: System only fires when exactly one balloon is detected
2. **Firing Delay**: 3-second cooldown between shots
3. **Alignment Check**: Target must be within tolerance before firing
4. **Manual Override**: All functions can be controlled manually
5. **Emergency Stop**: Immediate system shutdown with 'q' key

## Configuration

### Camera Settings
Modify in `main.py`:
```python
self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
```

### Servo Limits
Modify in `servo_control.py`:
```python
self.pan_min, self.pan_max = 0, 180
self.tilt_min, self.tilt_max = 45, 135  # Safety limits
```

### Detection Confidence
Modify in `detector.py`:
```python
self.confidence_threshold = 0.5  # Adjust as needed
```

## Custom Model Training

To train a custom YOLOv5 model for balloon detection:

1. **Collect balloon images** in various conditions
2. **Annotate images** using tools like LabelImg or Roboflow
3. **Train YOLOv5 model** following Ultralytics documentation
4. **Replace model path** in the system

## Troubleshooting

### Camera Issues
- Check camera index (try 0, 1, 2...)
- Verify camera permissions
- Test with `cv2.VideoCapture(0)`

### Servo Issues
- Check GPIO/pin connections
- Verify power supply (servos need 5V)
- Test servo movement manually

### Detection Issues
- Ensure good lighting conditions
- Check model confidence threshold
- Verify balloon is clearly visible

### Arduino Communication
- Check serial port name
- Verify baud rate (9600)
- Test Arduino code separately

## File Structure

```
balloon_targeting_system/
├── main.py              # Main application
├── detector.py          # YOLOv5 detection module
├── servo_control.py     # Servo control module
├── laser_control.py     # Laser control module
├── arduino_code.ino     # Arduino firmware
├── requirements.txt     # Python dependencies
└── README.md           # This file
```

## Performance Tips

1. **Use GPU acceleration** if available (CUDA)
2. **Optimize camera resolution** for your needs
3. **Adjust detection confidence** for better accuracy
4. **Use wired camera** for lower latency
5. **Close unnecessary applications** for better performance

## Safety Warning

⚠️ **IMPORTANT SAFETY NOTICE** ⚠️

This system is designed for educational and recreational purposes. When building and operating:

- Always follow local laws and regulations
- Use appropriate safety equipment
- Never aim at people, animals, or property
- Ensure safe firing range and backstop
- Test thoroughly in controlled environment
- Implement additional safety measures as needed

## License

This project is provided for educational purposes. Use responsibly and in accordance with local laws.

## Support

For issues or questions:
1. Check the troubleshooting section
2. Verify hardware connections
3. Test individual components separately
4. Review system logs for error messages

