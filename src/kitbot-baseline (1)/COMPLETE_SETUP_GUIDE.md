# Complete Robot Setup Guide

## 🚀 Maria Carrillo FIRST - Robot Setup & Usage Guide

This guide covers the complete setup and usage of the robot's systems including shooting, vision, odometry, cameras, and Shuffleboard integration.

## 📋 Table of Contents

1. [Robot Systems Overview](#robot-systems-overview)
2. [Controller Mapping](#controller-mapping)
3. [Shooting System](#shooting-system)
4. [Vision System (Limelight)](#vision-system-limelight)
5. [Odometry & Field Position](#odometry--field-position)
6. [Camera System](#camera-system)
7. [Shuffleboard Setup](#shuffleboard-setup)
8. [Troubleshooting](#troubleshooting)

---

## 🤖 Robot Systems Overview

### **Core Subsystems:**
- **DriveSubsystemSparkMax** - Drive motors with encoders
- **LimelightSubsystem** - Vision targeting and AprilTag detection
- **PulsingShooterSubsystem** - Shooting with pulsing pattern (3s ON / 0.5s OFF)
- **FuelSubsystemSparkMax** - Intake and fuel management
- **SimpleCameraSubsystem** - Single/dual camera URL publishing to SmartDashboard

### **Utility Systems:**
- **FieldPositionSystem** - NetworkTables-based position tracking
- **ShootingPositionManager** - Predefined shooting positions
- **ShuffleboardManager** - Organized Shuffleboard widgets
- **DualUSBCameraServer** - Multi-camera management system

---

## 🎮 Controller Mapping

### **Driver Controller (Port 0):**
- **Left Stick:** Arcade drive (default)
- **Right Stick:** Reset field position
- **LEFT BUMPER:** Odometry diagnostic
- **RIGHT BUMPER:** Odometry diagnostic (enhanced)
- **START Button:** Limelight connection test
- **BACK Button:** Continuous Limelight diagnostic

### **Operator Controller (Port 1):**
- **D-pad UP:** Select shooting position (Speaker positions)
- **D-pad RIGHT:** Next shooting position
- **D-pad DOWN:** Select shooting position (Stage positions)
- **D-pad LEFT:** Previous shooting position
- **X Button:** Shoot at selected position
- **B Button:** Simple auto-shoot
- **A Button:** Ball ejection (intake motor reverse only)
- **Y Button:** Toggle pulsing shooter
- **LEFT BUMPER:** Intake fuel
- **RIGHT BUMPER:** Continuous shooter

---

## 🏀 Shooting System

### **Features:**
- **Pulsing Shooter:** 3 seconds at 100% power, 0.5 seconds at 30% power
- **Position-Based Shooting:** 10 predefined shooting positions
- **Vision Integration:** Limelight targeting for accurate shooting
- **Parallel Operation:** Robot can move while shooting

### **Shooting Positions:**
1. **Speaker Center** - Direct speaker shot
2. **Speaker Left** - Speaker left side
3. **Speaker Right** - Speaker right side
4. **Stage Left** - Stage left side
5. **Stage Center** - Stage center
6. **Stage Right** - Stage right side
7. **Wing Left** - Wing left pickup
8. **Wing Center** - Wing center
9. **Wing Right** - Wing right side
10. **Amp Center** - Amplifier center

### **Shooting Commands:**
- **ShootAtPositionCommand:** Shoots at selected position with aiming
- **SimpleAutoShootCommand:** Basic auto-shoot with pulsing
- **SelectShootingPositionCommand:** D-pad position selection

---

## 👁 Vision System (Limelight)

### **Configuration:**
- **IP Address:** 10.57.28.11
- **Stream URL:** http://10.57.28.11:5800/stream.mjpg
- **NetworkTables Table:** limelight

### **Available Data:**
- **tv:** Target valid (0/1)
- **tx:** Horizontal offset (-27° to 27°)
- **ty:** Vertical offset (-20.5° to 20.5°)
- **ta:** Target area (0 to 100)
- **tid:** Target ID (AprilTag number)
- **ts:** Timestamp
- **tl:** Latency (ms)

### **Diagnostic Commands:**
- **LimelightTestCommand:** Comprehensive connection test
- **LimelightDiagnosticCommand:** Continuous status monitoring
- **VirtualLimelightTestCommand:** Simulated data testing

### **Troubleshooting:**
1. **No connection:** Check Ethernet cable and IP address
2. **No target:** Point at AprilTag, check lighting
3. **NetworkTables issues:** Press START button for test

---

## 📍 Odometry & Field Position

### **DriveSubsystem Odometry:**
- **Encoders:** DIO ports 0 and 2 (external)
- **Periodic Updates:** Automatic position tracking
- **SmartDashboard:** Real-time position data

### **FieldPositionSystem:**
- **NetworkTables Table:** Field
- **Position Tracking:** Robot X, Y, heading
- **Confidence:** Position accuracy (0.0 to 1.0)
- **Position Source:** VISION/ENCODERS

### **Available Data:**
```
Drive/Odometry/X: Robot X position
Drive/Odometry/Y: Robot Y position
Drive/Odometry/Heading: Robot heading in degrees
Drive/Encoder/Left: Left encoder distance
Drive/Encoder/Right: Right encoder distance

Field/RobotX: Field X position
Field/RobotY: Field Y position
Field/Confidence: Position confidence
Field/PositionSource: Position data source
```

### **Reset Commands:**
- **ResetFieldPositionCommand:** Reset to field center
- **OdometryDiagnosticCommand:** Comprehensive odometry status

---

## 📷 Camera System

### **SimpleCameraSubsystem:**
- **Dual Camera Support:** Can handle single or dual USB cameras
- **Pure SmartDashboard:** No complex CameraServer dependencies
- **URL-Based:** Direct MJPEG stream URLs
- **NetworkTables Integration:** Camera status monitoring
- **Configurable:** Switch between single/dual camera modes

### **Camera Options:**
- **Single Camera Mode:** One USB camera + Limelight
- **Dual Camera Mode:** Two USB cameras + Limelight
- **Configuration:** Set in `CameraConfig.USE_DUAL_CAMERAS`

### **Camera URLs:**
```
Limelight: http://172.22.11.2:5800/stream.mjpg
Primary USB: http://roboRIO-5728.local:1181/stream.mjpg
Secondary USB: http://roboRIO-5728.local:1182/stream.mjpg (dual mode)
```

### **SmartDashboard Entries:**
```
# Single Camera Mode
Camera/Limelight_URL: Limelight stream URL
Camera/Limelight_Available: Limelight status
Camera/USB_URL: USB camera URL
Camera/USB_Available: USB camera status

# Dual Camera Mode
Camera/Primary_USB_URL: Primary camera URL
Camera/Primary_USB_Available: Primary camera status
Camera/Secondary_USB_URL: Secondary camera URL
Camera/Secondary_USB_Available: Secondary camera status
```

### **Enabling Dual Camera Mode:**
1. **Edit CameraConfig.java:**
   ```java
   public static final boolean USE_DUAL_CAMERAS = true;
   ```
2. **Update RobotContainer.java:**
   ```java
   // Replace USBCameraServer.initialize() with:
   DualUSBCameraServer.initialize();
   ```
3. **Update SimpleCameraSubsystem:**
   ```java
   // Replace new SimpleCameraSubsystem() with:
   new SimpleCameraSubsystem(true);
   ```

### **Shuffleboard Setup:**
#### **Single Camera Mode:**
1. **Add Camera widget**
2. **Select "Camera Server"** source
3. **Choose "DriverCamera"**
4. **Add second Camera widget**
5. **Select "Custom URL"** source
6. **Enter Limelight URL:** http://172.22.11.2:5800/stream.mjpg

#### **Dual Camera Mode:**
1. **Add Camera widget**
2. **Select "Camera Server"** source
3. **Choose "DriverCamera"** (primary)
4. **Add second Camera widget**
5. **Select "Camera Server"** source
6. **Choose "SecondaryCamera"** (secondary)
7. **Add third Camera widget**
8. **Select "Custom URL"** source
9. **Enter Limelight URL:** http://172.22.11.2:5800/stream.mjpg

---

## 📱 Shuffleboard Setup

### **Organized Tabs:**
- **Driver Tab:** Main driver interface
- **Cameras Tab:** Camera monitoring
- **Debug Tab:** System debugging

### **Driver Tab Widgets:**
- **Vision Data:** Target detection, offsets, distance
- **Odometry:** Robot position, heading, encoders
- **Field Position:** GPS-like position tracking
- **Shooting:** Position selection and confidence
- **Fuel System:** Intake, feeder, shooter status

### **Cameras Tab Widgets:**
- **Camera Status:** USB & Limelight availability
- **Stream URLs:** Direct camera feed access

### **Debug Tab Widgets:**
- **Error Messages:** System error tracking
- **Model Status:** 3D model publishing
- **Test Results:** Diagnostic outputs

### **Widget Types:**
- **BooleanBox:** Green/Red status indicators
- **NumberBar:** Visual bar graphs with limits
- **NumberSlider:** Interactive sliders
- **TextView:** Text information display

---

## 🔧 Troubleshooting

### **🚨 Common Issues:**

#### **Robot Not Moving:**
1. **Check DriveSubsystem:** Look for encoder errors in console
2. **Verify motors:** Check motor controller connections
3. **Test joysticks:** Verify controller is connected

#### **Shooting Issues:**
1. **Robot paralyzed while shooting:** ✅ FIXED - DriveSubsystem requirement removed
2. **Shooter not firing:** Check PulsingShooterSubsystem status
3. **No target detection:** Point Limelight at AprilTag

#### **Vision Issues:**
1. **Limelight not connected:** Check IP address (10.57.28.11)
2. **No target detected:** Check lighting and AprilTag visibility
3. **NetworkTables errors:** Press START button for test

#### **Camera Issues:**
1. **Camera not showing:** ✅ FIXED - Use URL-based approach
2. **USB camera problems:** Check physical connection
3. **Limelight stream:** Test URL in browser first
4. **Dual camera not working:** Check CameraConfig.USE_DUAL_CAMERAS setting
5. **Second camera not detected:** Try different device numbers (0, 1, 2, 3)
6. **Camera stream URL wrong:** Verify team number 5728 in configuration

#### **Odometry Issues:**
1. **Position stuck at (0,0):** Check robot movement and encoders
2. **Position jumping:** Check for wheel slippage

### **🛠️ Diagnostic Commands:**

#### **Driver Controller:**
- **RIGHT BUMPER:** Odometry diagnostic
- **START Button:** Limelight connection test
- **BACK Button:** Continuous Limelight monitoring

#### **Console Output:**
- **Camera Status:** Every 10 seconds
- **Odometry Updates:** Every 1 second
- **Limelight Connection:** Real-time status

#### **SmartDashboard Values:**
- **Drive/Odometry:** Position and heading data
- **Limelight/**: Vision targeting data
- **Camera/**: Camera status and URLs
- **Field/**: Field position tracking

---

## ✅ Quick Start Checklist

### **Before Competition:**
1. ✅ **Compile code:** `./gradlew compileJava`
2. ✅ **Test controllers:** Verify all buttons work
3. ✅ **Test Limelight:** Point at AprilTag, check detection
4. ✅ **Test shooter:** Verify pulsing pattern works
5. ✅ **Test odometry:** Move robot, check position updates

### **During Competition:**
1. ✅ **Connect Shuffleboard:** Team 5728
2. ✅ **Add camera widgets:** Use Limelight URL
3. ✅ **Monitor status:** Check all widgets update
4. ✅ **Use diagnostics:** Press buttons if issues arise

### **After Competition:**
1. ✅ **Review logs:** Check console for errors
2. ✅ **Backup code:** Commit changes to git
3. ✅ **Document issues:** Note any problems found

---

## 🎯 Key Features Working

- ✅ **Parallel shooting + driving:** Robot can move while shooting
- ✅ **Pure NetworkTables cameras:** Simple, reliable camera system
- ✅ **Organized Shuffleboard:** Clean, intuitive driver interface
- ✅ **Comprehensive diagnostics:** Built-in troubleshooting tools
- ✅ **Position-based shooting:** 10 predefined shooting positions
- ✅ **Real-time odometry:** Accurate position tracking
- ✅ **Vision integration:** Limelight targeting with confidence

---

## 📚 Additional Resources

### **Documentation Files:**
- `CONTROLLER_MAPPING_GUIDE.md` - Detailed controller mapping
- `FIELD_POSITION_GUIDE.md` - Field position system details
- `LIMELIGHT_TROUBLESHOOTING.md` - Limelight troubleshooting
- `ODOMETRY_TROUBLESHOOTING.md` - Odometry system debugging
- `SHUFFLEBOARD_SETUP_GUIDE.md` - Complete Shuffleboard setup

### **Code Structure:**
- `src/main/java/frc/robot/subsystems/` - Core robot subsystems
- `src/main/java/frc/robot/commands/` - Command-based operations
- `src/main/java/frc/robot/utils/` - Utility classes
- `src/main/java/frc/robot/config/` - Configuration management

---

## 🚀 Ready for Competition!

The robot is fully configured and ready for competition with:
- **Reliable shooting system** with parallel movement
- **Working vision system** with Limelight integration
- **Accurate odometry** with field position tracking
- **Simple camera system** with Shuffleboard integration
- **Comprehensive diagnostics** for quick troubleshooting

**Good luck, Team 5728! 🤖**
