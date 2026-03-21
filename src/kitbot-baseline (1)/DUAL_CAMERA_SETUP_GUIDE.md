# Dual Camera System Setup Guide

## 🎯 Overview

Your robot now supports **dual USB cameras** for complete driver visibility! This system allows you to display **both cameras simultaneously** in Shuffleboard, giving you:

- **Primary Camera**: Driver view (forward-facing)
- **Secondary Camera**: Intake/goal view (backward/downward-facing)
- **Limelight**: Vision targeting (always available)

## 🚀 Quick Setup

### 1. Camera Configuration
Your `CameraConfig.java` is already configured:
```java
public static final boolean USE_DUAL_CAMERAS = true;
public static final int PRIMARY_USB_CAMERA_DEVICE = 1;   // Driver camera
public static final int SECONDARY_USB_CAMERA_DEVICE = 0; // Intake camera
```

### 2. Shuffleboard Setup

#### **Add Primary Camera:**
1. Open Shuffleboard and connect to Team 5728
2. Click **'+'** to add widget
3. Select **'Camera Server'**
4. Choose **'DriverCamera'** (Primary)
5. Resize and position on left side

#### **Add Secondary Camera:**
1. Click **'+'** again to add second widget
2. Select **'Camera Server'**
3. Choose **'SecondaryCamera'** (Secondary)
4. Resize and position on right side

#### **Add Limelight:**
1. Click **'+'** again to add third widget
2. Select **'Camera Server'**
3. Choose **'Custom URL'**
4. Enter: `http://172.22.11.2:5800/stream.mjpg`

### 3. Recommended Layout

```
┌─────────────────┬─────────────────┐
│  Primary Camera │ Secondary Camera│
│  (Driver View)  │  (Intake View)  │
├─────────────────┼─────────────────┤
│   Limelight     │   Dashboard     │
│  (Targeting)    │  (Telemetry)    │
└─────────────────┴─────────────────┘
```

## 🔧 Hardware Setup

### USB Camera Connections:
- **Primary Camera (Device 1)**: Forward-facing driver camera
- **Secondary Camera (Device 0)**: Downward/backward intake camera
- Both cameras connect to roboRIO USB ports

### Camera Recommendations:
- **Logitech C270** or **C920** (good performance, reliable)
- **Microsoft LifeCam** (alternative option)
- Ensure both cameras are recognized on computer first

## 📱 Camera URLs

### Direct Access (for testing):
- **Primary**: `http://roboRIO-5728.local:1181/stream.mjpg`
- **Secondary**: `http://roboRIO-5728.local:1182/stream.mjpg`
- **Limelight**: `http://172.22.11.2:5800/stream.mjpg`

### SmartDashboard Keys:
- `Primary_USB_Camera_URL`: Primary camera stream
- `Secondary_USB_Camera_URL`: Secondary camera stream
- `Limelight_URL`: Limelight stream

## 🎮 Controller Integration

Your existing controller buttons work with dual cameras:

### Driver Controller:
- **A Button**: Camera switching (single camera mode only)
- **B Button**: Movement inversion toggle

### Operator Controller:
- **Left Trigger**: Camera switching (single camera mode only)

*Note: In dual camera mode, both cameras are always visible, so switching isn't needed*

## 🛠️ Troubleshooting

### **Camera Not Showing:**
1. **Check USB Connections**: Ensure cameras are securely connected to roboRIO
2. **Verify Device Numbers**: Try swapping device numbers (0 ↔ 1) in `CameraConfig.java`
3. **Test on Computer**: Connect cameras to computer first to verify they work
4. **Check roboRIO Web Interface**: 
   - Navigate to `http://roboRIO-5728.local`
   - Check if cameras are detected under USB devices

### **Only One Camera Works:**
1. **Power Issue**: USB ports may not provide enough power
2. **Device Conflict**: Try different device numbers (0, 1, 2, 3)
3. **Camera Compatibility**: Some cameras don't work well together

### **Poor Video Quality:**
1. **Reduce Resolution**: Lower `CAMERA_WIDTH` and `CAMERA_HEIGHT` in `CameraConfig.java`
2. **Reduce FPS**: Lower `CAMERA_FPS` to 15 or 20
3. **Check Cable Quality**: Use quality USB cables

### **Network Issues:**
1. **Team Number**: Ensure team number 5728 is correctly configured
2. **Network Connection**: Verify roboRIO is connected to driver station
3. **Firewall**: Check if firewall is blocking camera streams

## 📊 Status Monitoring

### Console Output:
The system prints status every 10 seconds:
```
=== Camera Status Update ===
Camera Mode: DUAL CAMERA
  Primary USB Camera: AVAILABLE
  Secondary USB Camera: AVAILABLE
  Primary URL: http://roboRIO-5728.local:1181/stream.mjpg
  Secondary URL: http://roboRIO-5728.local:1182/stream.mjpg
  Limelight: AVAILABLE
  Limelight URL: http://172.22.11.2:5800/stream.mjpg
=============================
```

### SmartDashboard Values:
- `DualCameras/Initialized`: System initialization status
- `DualCameras/PrimaryConnected`: Primary camera connection
- `DualCameras/SecondaryConnected`: Secondary camera connection
- `DualCameras/Status`: Overall system status

## 🔄 Switching Between Single/Dual Mode

### **Enable Dual Cameras:**
```java
// In CameraConfig.java
public static final boolean USE_DUAL_CAMERAS = true;

// In RobotContainer.java
private final SimpleCameraSubsystem cameraServerSubsystem = new SimpleCameraSubsystem(true);
```

### **Enable Single Camera:**
```java
// In CameraConfig.java
public static final boolean USE_DUAL_CAMERAS = false;

// In RobotContainer.java
private final SimpleCameraSubsystem cameraServerSubsystem = new SimpleCameraSubsystem(false);
```

## 🎯 Competition Tips

### **Pre-Match Checklist:**
1. ✅ Verify both cameras show in Shuffleboard
2. ✅ Check camera feeds are clear and stable
3. ✅ Confirm Limelight targeting works
4. ✅ Test camera switching if using single mode
5. ✅ Monitor console for camera status messages

### **During Match:**
- **Driver**: Focus on primary camera for navigation
- **Operator**: Monitor secondary camera for intake/game pieces
- **Both**: Use Limelight for precise targeting

### **If Camera Fails:**
- System automatically handles camera failures
- Check console for specific error messages
- Can continue with remaining cameras
- Use Limelight as backup targeting

## 🚀 Advanced Features

### **Custom Camera Placement:**
- Adjust camera positions for optimal viewing angles
- Consider mounting for minimal vibration
- Ensure cameras don't interfere with game pieces

### **Performance Optimization:**
- Lower resolution/FPS if network bandwidth is limited
- Use quality USB cables for reliable connections
- Monitor CPU usage on roboRIO

### **Integration with Autonomous:**
- Cameras continue working during autonomous
- Use Limelight for vision-based auto routines
- Monitor camera status for debugging

---

**Your dual camera system is now ready for competition! 🎉**

This gives you **complete situational awareness** with:
- **Forward visibility** for driving
- **Rear/intake visibility** for game piece handling  
- **Vision targeting** for precise shooting

Good luck, Team 5728! 🤖
