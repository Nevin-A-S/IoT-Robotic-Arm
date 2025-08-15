# IoT Robotic Arm for Automated Waste Sorting

An intelligent robotic arm system that uses computer vision and machine learning to automatically detect, pick up, and sort waste materials into appropriate disposal bins. The system combines YOLO object detection with precise servo control to create an automated waste management solution.

## 🌟 Features

- **Real-time Object Detection**: Uses YOLO model to identify different waste materials (paper, metal, plastic)
- **Automated Sorting**: Intelligently sorts detected objects into designated disposal bins
- **Distance Measurement**: Ultrasonic sensor for precise object positioning
- **Multi-Servo Control**: Smooth movement control for 9 servos with PWM driver
- **Camera Integration**: Real-time video processing with OpenCV
- **Object Centering**: Automatic centering of objects before pickup
- **Serial Communication**: Robust communication between Python backend and Arduino
- **Configurable Settings**: Easily adjustable detection thresholds and servo positions
- **Comprehensive Logging**: Detailed logging system for debugging and monitoring

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Camera Feed   │───▶│  Python Backend  │───▶│   Arduino UNO   │
└─────────────────┘    │                  │    │                 │
                       │  - YOLO Detection│    │ - Servo Control │
┌─────────────────┐    │  - Object Track  │    │ - Distance Sens │
│  YOLO Weights   │───▶│  - Path Planning │    │ - PWM Driver    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
                                │                        │
                                ▼                        ▼
                       ┌──────────────────┐    ┌─────────────────┐
                       │   Control Logic  │    │  Robotic Arm    │
                       │  - Pickup Seq    │    │  - 6 Servos     │
                       │  - Disposal Seq  │    │  - Gripper      │
                       └──────────────────┘    │  - Base Rotation│
                                               └─────────────────┘
```

## 🔧 Hardware Requirements

### Essential Components

- **Arduino UNO** - Main microcontroller
- **Adafruit PWM Servo Driver (PCA9685)** - Controls up to 16 servos
- **6x Servo Motors** - For robotic arm joints and gripper
- **HC-SR04 Ultrasonic Sensor** - Distance measurement
- **ESP 32 Camera with wide angle lens** - Object detection
- **Jumper Wires & Breadboard** - Connections
- **Power Supply(5V 10 AMP SMPS Recommended)** - 5V for servos (external recommended)

### Wiring Connections

```
Arduino UNO ↔ PWM Servo Driver (PCA9685)
- VCC ↔ 5V
- GND ↔ GND
- SDA ↔ A4
- SCL ↔ A5

Arduino UNO ↔ Ultrasonic Sensor (HC-SR04)
- Digital Pin 7 ↔ TRIG
- Digital Pin 8 ↔ ECHO
- 5V ↔ VCC
- GND ↔ GND

Servos ↔ PWM Driver
- Servo 1-7 ↔ Channels 0-5 on PCA9685
```

## 💻 Software Requirements

### Python Dependencies

- Python 3.8 or higher
- OpenCV (`opencv-python`)
- Ultralytics YOLO (`ultralytics`)
- NumPy (`numpy`)
- PySerial (`pyserial`)

### Arduino Libraries

- Wire.h (built-in)
- Adafruit_PWMServoDriver.h

## 🚀 Installation & Setup

### 1. Clone the Repository

```bash
git clone https://github.com/Nevin-A-S/Iot-Robotic-arm.git
cd IoT-Robotic-Arm
```

### 2. Python Environment Setup

```bash
# Create virtual environment (recommended)
python -m venv venv

# Activate virtual environment
# On Windows:
venv\Scripts\activate
# On macOS/Linux:
source venv/bin/activate


# Install package in development mode
pip install -e .
```

### 3. Arduino Setup

1. **Install Arduino IDE** from [arduino.cc](https://www.arduino.cc/en/software)

2. **Install Required Libraries**:

   - Open Arduino IDE
   - Go to `Tools > Manage Libraries`
   - Search and install: `Adafruit PWM Servo Driver Library`

3. **Upload Arduino Code**:
   ```bash
   # Open Arduino_code/Servo.ino in Arduino IDE
   # Select your Arduino board and COM port
   # Click Upload
   ```

### 4. Hardware Assembly

1. Connect all components according to the wiring diagram
2. Ensure proper power supply for servos (external 5V recommended)
3. Test individual servo movements using Arduino Serial Monitor

### 5. YOLO Model Setup

```bash
# The project includes pre-trained weights in src/yolo/weights/
# Ensure best.pt and last.pt are present
# If missing, train your own model or download compatible YOLO weights
```

## ⚙️ Configuration

### Camera Settings

Edit `src/config/config.py`:

```python
# Camera settings
CAMERA_INDEX = "http://192.168.0.176:81/stream"  # IP camera URL or 0 for USB camera
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
```

### Servo Positions

Modify servo positions in `src/controller/servo_controller.py`:

```python
self.positionData = {
    "17": "S2A58S3A150S4A115S5A105",  # Distance-based positions
    "18": "S2A52S3A145S4A115S5A105",
    # Add more positions as needed
    "paperDisposal": "S1A70S2A120S3A40",
    "plasticDisposal": "S1A50S2A120S3A40",
    "metalDisposal": "S1A100S2A120S3A40",
}
```

### Detection Settings

Adjust detection parameters:

```python
# YOLO model settings
CONFIDENCE_THRESHOLD = 0.4
DETECTION_CONFIDENCE_THRESHOLD = 0.6
CENTER_THRESHOLD = 10  # Pixels from center
```

### Serial Communication

Update COM port in config:

```python
SERVO_PORT = "COM3"  # Change to your Arduino's COM port
```

## 🎮 Usage

### 1. Basic Operation

```bash
# Navigate to project directory
cd IoT-Robotic-Arm

# Run the main application
python src/main.py
```

### 2. System Workflow

1. **Initialization**: System initializes camera, YOLO model, and Arduino connection
2. **Object Detection**: Camera continuously scans for objects
3. **Object Centering**: Base servo rotates to center detected object
4. **Distance Measurement**: Ultrasonic sensor measures distance to object
5. **Pickup Sequence**: Arm moves to appropriate position and grabs object
6. **Sorting**: Object is moved to correct disposal bin based on classification
7. **Return**: Arm returns to neutral position and continues scanning

### 3. Manual Testing

```bash
# Test individual components:

# Test servo movements (Arduino Serial Monitor)
S1A90    # Move servo 1 to 90 degrees
S1A90S2A45S3A180    # Move multiple servos

# Test distance measurement
DIST     # Get immediate distance
GET      # Get average distance over 60 seconds
```

## 📁 Project Structure

```
IoT-Robotic-Arm/
├── README.md                          # Project documentation
├── requirements.txt                   # Python dependencies
├── setup.py                          # Package setup configuration
├── ruff.toml                         # Code linting configuration
│
├── Arduino_code/                     # Arduino firmware
│   └── Servo.ino                    # Main Arduino control code
│
├── Servo_test/                       # Arduino testing code
│   └── Servo_test.ino               # Servo testing utilities
│
└── src/                             # Python source code
    ├── __init__.py                  # Package initialization
    ├── main.py                      # Main application entry point
    │
    ├── config/                      # Configuration files
    │   ├── __init__.py
    │   └── config.py               # System configuration parameters
    │
    ├── controller/                  # Control logic
    │   ├── __init__.py
    │   ├── robotic_arm_controller.py  # Main arm control logic
    │   └── servo_controller.py        # Servo communication & control
    │
    ├── detector/                    # Object detection
    │   ├── __init__.py
    │   ├── object_detector.py       # YOLO-based object detection
    │   └── bbox_utils.py           # Bounding box utilities
    │
    ├── utils/                       # Utility functions
    │   ├── __init__.py
    │   └── logger.py               # Logging configuration
    │
    ├── yolo/                        # YOLO model files
    │   ├── __init__.py
    │   ├── args.yaml               # YOLO training arguments
    │   └── weights/                # Model weights
    │       ├── best.pt             # Best trained model
    │       └── last.pt             # Latest checkpoint
    │
    └── logs/                        # Log files (generated at runtime)
```

### Key Files Explained

- **`src/main.py`**: Application entry point, initializes and starts the robotic arm controller
- **`src/controller/robotic_arm_controller.py`**: Core control logic, manages detection-to-action pipeline
- **`src/controller/servo_controller.py`**: Handles serial communication with Arduino and servo positioning
- **`src/detector/object_detector.py`**: YOLO-based object detection and classification
- **`src/config/config.py`**: Centralized configuration for all system parameters
- **`Arduino_code/Servo.ino`**: Arduino firmware for servo control and sensor reading

## 🔧 Troubleshooting

### Common Issues

#### 1. Camera Connection Issues

```bash
# Error: Failed to open camera
# Solutions:
- Check camera URL/index in config.py
- Ensure camera is not being used by another application
- For IP cameras, verify network connection and URL format
- For USB cameras, try different indices (0, 1, 2, etc.)
```

#### 2. Arduino Communication Problems

```bash
# Error: Serial port connection failed
# Solutions:
- Verify correct COM port in config.py
- Check Arduino is properly connected via USB
- Ensure Arduino IDE Serial Monitor is closed
- Try different baud rates (9600 is default)
- Check if Arduino code is uploaded correctly
```

#### 3. Servo Movement Issues

```bash
# Error: Servos not responding or moving incorrectly
# Solutions:
- Check power supply (servos need adequate current)
- Verify PWM driver connections (SDA, SCL, VCC, GND)
- Test individual servos using Arduino Serial Monitor
- Calibrate servo positions in positionData dictionary
- Check for loose connections
```

#### 4. Object Detection Problems

```bash
# Error: No objects detected or poor accuracy
# Solutions:
- Adjust CONFIDENCE_THRESHOLD in config.py
- Ensure proper lighting conditions
- Check if YOLO weights are present in src/yolo/weights/
- Verify camera focus and positioning
- Retrain model with your specific objects if needed
```

#### 5. Distance Measurement Issues

```bash
# Error: Inaccurate or no distance readings
# Solutions:
- Check ultrasonic sensor wiring (TRIG pin 7, ECHO pin 8)
- Ensure sensor has clear line of sight
- Verify 5V power supply to sensor
- Test sensor independently using DIST command
```

### Debug Mode

Enable detailed logging by setting in `config.py`:

```python
LOG_LEVEL = "DEBUG"
```

### Performance Optimization

- Use external power supply for servos
- Optimize YOLO model size for faster inference
- Adjust camera resolution for better performance
- Fine-tune servo movement speeds

## 🚀 Future Scope & Enhancements

### 📱 IoT Integration & Smart Monitoring

- **Smart Bin Monitoring**: Implement ultrasonic sensors in disposal bins to monitor fill levels
- **Mobile Alerts**: Real-time notifications to mobile phones when bins reach capacity
- **Cloud Dashboard**: Web-based monitoring system for multiple robotic arms
- **Predictive Analytics**: AI-powered waste generation patterns and optimization
- **Remote Control**: Mobile app for manual override and system control

### 🧠 Advanced AI & Vision

- **Vision Language Models (VLMs)**:
  - Integration with models like GPT-4V or LLaVA for zero-shot object recognition
  - Eliminate need for training on specific object types
  - Natural language object descriptions and sorting instructions
- **Voice-Activated AI Agent**:
  - Speech recognition for voice commands ("Pick up the bottle", "Sort plastic items")
  - Text-to-speech feedback for system status
  - Integration with virtual assistants (Alexa, Google Assistant)
- **Advanced Computer Vision**:
  - 3D object reconstruction for better manipulation
  - Multi-camera setup for 360° object analysis
  - Real-time object tracking and prediction

### 🤖 Enhanced Robotics & Control

- **Dynamic Object Picking with Inverse Kinematics**:
  - Mathematical calculation of joint angles for any target position
  - Smooth trajectory planning and collision avoidance
  - Adaptive gripper control based on object shape and material
- **Machine Learning Integration**:
  - Reinforcement learning for optimal pickup strategies
  - Continuous improvement through success/failure feedback
  - Adaptive positioning based on object characteristics
- **Advanced Manipulation**:
  - Force feedback sensors for delicate object handling
  - Multi-gripper system for different object types
  - Collaborative multi-arm coordination

### 🌐 Connectivity & Integration

- **IoT Ecosystem**:
  - MQTT protocol for device communication
  - Integration with smart building systems
  - Environmental sensors (air quality, temperature)
- **Edge Computing**:
  - Local AI processing for reduced latency
  - Offline operation capabilities
  - Edge-to-cloud synchronization

### 📊 Data Analytics & Optimization

- **Advanced Analytics**:
  - Waste composition analysis and reporting
  - Efficiency metrics and performance optimization
  - Environmental impact assessment
- **Predictive Maintenance**:
  - Servo wear prediction and replacement scheduling
  - System health monitoring and diagnostics
  - Automated calibration and adjustment

### 🔧 Hardware Enhancements

- **Modular Design**:
  - Interchangeable end effectors for different tasks
  - Scalable servo configurations
  - Hot-swappable components
- **Advanced Sensors**:
  - LIDAR for 3D environment mapping
  - Thermal imaging for material identification
  - Chemical sensors for hazardous material detection

### Implementation Roadmap

```
Phase 1 (Current): Basic object detection and sorting
Phase 2: IoT integration and mobile alerts
Phase 3: VLM integration and voice control
Phase 4: Inverse kinematics and advanced manipulation
Phase 5: Full ecosystem integration and analytics
```

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines

- Follow PEP 8 style guidelines
- Add comprehensive docstrings
- Include unit tests for new features
- Update documentation as needed
- Test on actual hardware before submitting

### Priority Areas for Contribution

- **IoT Integration**: Mobile app development and cloud connectivity
- **AI Enhancement**: VLM integration and voice recognition
- **Robotics**: Inverse kinematics implementation
- **Hardware**: Sensor integration and modular design
- **Analytics**: Data visualization and reporting tools

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Ultralytics** for the YOLO object detection framework
- **Adafruit** for the PWM servo driver library
- **OpenCV** community for computer vision tools
- **Arduino** community for embedded development resources

## 📞 Support

If you encounter any issues or have questions:

1. Check the troubleshooting section above
2. Search existing issues in the repository
3. Create a new issue with detailed description and logs
4. Include your hardware setup and configuration details

---

**Happy Building! 🤖**
