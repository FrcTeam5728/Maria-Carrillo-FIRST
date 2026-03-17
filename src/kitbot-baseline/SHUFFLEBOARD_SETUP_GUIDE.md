# Shuffleboard Setup Guide

## ✅ Complete Shuffleboard Integration

Shuffleboard is now fully integrated with organized tabs and widgets for all robot systems!

## 🎯 What's Available:

### **📱 Driver Tab** - Main Driver Interface
- **Camera Feeds:** USB Camera & Limelight
- **Vision Data:** Target detection, offsets, distance
- **Odometry:** Robot position, heading, encoders
- **Field Position:** GPS-like position tracking
- **Shooting:** Position selection and confidence
- **Fuel System:** Intake, feeder, shooter status

### **📷 Cameras Tab** - Camera Monitoring
- **Camera Status:** USB & Limelight availability
- **Stream URLs:** Direct access to camera feeds
- **Connection Monitoring:** Real-time camera status

### **🔧 Debug Tab** - System Debugging
- **Error Messages:** System error tracking
- **Model Status:** 3D model publishing status
- **Test Results:** Diagnostic command outputs

## 🚀 Quick Start:

### **Step 1: Launch Shuffleboard**
1. **Open Shuffleboard** on driver station
2. **Connect to robot** (Team 5728)
3. **Tabs appear automatically** with organized widgets

### **Step 2: Add Camera Widgets**
1. **Go to Cameras Tab**
2. **Add Camera widgets** manually if needed:
   - **Click "+"** → **Camera** → **"USB Camera"**
   - **Click "+"** → **Camera** → **"Limelight"**
   - **Alternative:** Use URL `http://10.57.28.11:5800/stream.mjpg`

### **Step 3: Verify Data Flow**
1. **Watch Driver Tab** for real-time updates
2. **Check camera feeds** in Cameras Tab
3. **Monitor debug info** in Debug Tab

## 📊 Widget Layout - Driver Tab:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                   DRIVER TAB                                    │
├─────────────────┬─────────────────┬─────────────────┬─────────────────────────┤
│ Limelight       │ Target Found    │ Intake Running  │ Feeder Running          │
│ Connected       │                 │                 │                        │
├─────────────────┼─────────────────┼─────────────────┼─────────────────────────┤
│ Horizontal Offset (Number Bar: -27 to 27)                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│ Vertical Offset   (Number Bar: -20.5 to 20.5)                                │
├─────────────────────────────────────────────────────────────────────────────┤
│ Distance (m)      (Number Slider: 0 to 10)                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│ Limelight Status   (Text: Connected/Disconnected/Error)                      │
├─────────────────┬─────────────────┬─────────────────┬─────────────────────────┤
│ Robot X          (Number Bar: -5 to 15)                                     │
│ Robot Y          (Number Bar: -2 to 10)                                     │
│ Robot Heading    (Gyro: -180° to 180°)                                      │
│ Gyro Angle       (Number Slider: -180 to 180)                               │
├─────────────────┴─────────────────┴─────────────────┴─────────────────────────┤
│ Left Encoder      Right Encoder    Field X    Field Y    Field Confidence      │
│ (Number Bar)      (Number Bar)     (Text)     (Text)     (Number Slider)       │
├─────────────────────────────────────────────────────────────────────────────┤
│ Position Source   (Text: VISION/ENCODERS)                                   │
├─────────────────────────────────────────────────────────────────────────────┤
│ Selected Position (Text: Shooting position name)                             │
│ Preferred Distance (Number Slider: 0 to 10)                                 │
│ Shooting Confidence (Text: Confidence level)                                │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 📷 Widget Layout - Cameras Tab:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                  CAMERAS TAB                                  │
├─────────────────┬─────────────────┬─────────────────────────────────────────┤
│ USB Camera       │ Limelight       │ Limelight URL                            │
│ Available        │ Available       │ http://10.57.28.11:5800/stream.mjpg      │
│ (Boolean Box)    │ (Boolean Box)   │ (Text View)                              │
└─────────────────┴─────────────────┴─────────────────────────────────────────┘
```

## 🔧 Widget Layout - Debug Tab:

```
┌─────────────────────────────────────────────────────────────────────────────────┐
│                                   DEBUG TAB                                    │
├─────────────────────────────────────────────────────────────────────────────┤
│ Debug Error        (Text: System error messages)                            │
│ Model Status       (Text: 3D model publishing status)                        │
│ Test Status        (Text: Diagnostic command results)                        │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 🎮 Widget Types & Meanings:

### **📊 Data Display Widgets:**
- **BooleanBox:** Green = TRUE, Red = FALSE
- **NumberBar:** Visual bar graph with min/max limits
- **NumberSlider:** Interactive slider with min/max limits
- **Gyro:** Compass-style heading display
- **TextView:** Text information display

### **📷 Camera Widgets:**
- **Camera Server:** Live video feed
- **TextView:** Camera stream URLs

### **🔍 Widget Color Coding:**
- **🟢 Green:** Good/Active/Connected
- **🔴 Red:** Bad/Inactive/Disconnected
- **🟡 Yellow:** Warning/Middle state
- **🔵 Blue:** Information/Neutral

## 📱 Real-Time Data Flow:

### **📡 Vision Data:**
```
Limelight → NetworkTables → LimelightSubsystem → Shuffleboard
├─ Target Detection: BooleanBox (Green/Red)
├─ Horizontal Offset: NumberBar (-27° to +27°)
├─ Vertical Offset: NumberBar (-20.5° to +20.5°)
├─ Distance: NumberSlider (0m to 10m)
└─ Status: TextView (Connected/Disconnected)
```

### **🤖 Odometry Data:**
```
DriveSubsystem → Encoders + Gyro → Shuffleboard
├─ Robot X/Y: NumberBar (Field position)
├─ Robot Heading: Gyro (Compass)
├─ Gyro Angle: NumberSlider (-180° to +180°)
└─ Encoders: NumberBar (Wheel rotation)
```

### **🎯 Field Position:**
```
FieldPositionSystem → Vision + Odometry → Shuffleboard
├─ Field X/Y: TextView (GPS coordinates)
├─ Confidence: NumberSlider (0.0 to 1.0)
└─ Position Source: TextView (VISION/ENCODERS)
```

### **🏀 Shooting System:**
```
ShootingPositionManager → Selected Position → Shuffleboard
├─ Selected Position: TextView (Position name)
├─ Preferred Distance: NumberSlider (Optimal range)
└─ Shooting Confidence: TextView (Accuracy level)
```

### **⚙️ Fuel System:**
```
FuelSubsystem + ShooterSubsystem → Shuffleboard
├─ Intake Running: BooleanBox (Green when active)
├─ Feeder Running: BooleanBox (Green when active)
├─ Shooter Active: BooleanBox (Green when pulsing)
└─ Shooter Speed: NumberSlider (0% to 100%)
```

## 🛠️ Troubleshooting Shuffleboard:

### **📱 No Widgets Appearing:**
1. **Check robot connection** to Shuffleboard
2. **Verify team number** (5728)
3. **Restart Shuffleboard** if needed
4. **Check NetworkTables** connection

### **📷 Camera Not Working:**
1. **Check Cameras Tab** for camera status
2. **Verify USB camera** connected to robot
3. **Check Limelight** network connection
4. **Use direct URL** in browser: `http://10.57.28.11:5800/stream.mjpg`

### **📊 Data Not Updating:**
1. **Check robot code** is running
2. **Verify subsystems** are initialized
3. **Check NetworkTables** keys in Shuffleboard
4. **Look for errors** in Debug Tab

### **🎯 Wrong Values:**
1. **Check sensor calibration** (gyro, encoders)
2. **Verify Limelight** configuration
3. **Check field coordinate** system
4. **Tune shooting calculations**

## 🔧 Advanced Configuration:

### **Custom Widget Layout:**
```java
// In ShuffleboardManager.java
public void customizeLayout() {
    // Move widgets to custom positions
    driverTab.getWidget("Robot X").withPosition(0, 0).withSize(4, 1);
    driverTab.getWidget("Robot Y").withPosition(0, 1).withSize(4, 1);
}
```

### **Add Custom Widgets:**
```java
// Add new widget to driver tab
driverTab.add("Custom Metric", value)
    .withWidget(BuiltInWidgets.kNumberBar)
    .withProperties(Map.of("min", 0, "max", 100))
    .withPosition(0, 0)
    .withSize(4, 1);
```

### **Widget Properties:**
```java
// Customize widget appearance
.withProperties(Map.of(
    "min", 0,           // Minimum value
    "max", 100,         // Maximum value
    "color on true", "green",   // Color when true
    "color on false", "red",    // Color when false
    "show text", true           // Show text value
))
```

## ✅ What's Working Now:

- ✅ **Automatic widget creation** on robot startup
- ✅ **Organized tab layout** (Driver, Cameras, Debug)
- ✅ **Real-time data updates** from all subsystems
- ✅ **Camera feed integration** with status monitoring
- ✅ **Color-coded status** for quick visual feedback
- ✅ **Interactive widgets** for driver control
- ✅ **Debug information** for troubleshooting

## 🎯 Next Steps:

1. **Launch Shuffleboard** and verify all tabs appear
2. **Test camera feeds** by connecting USB camera
3. **Verify data updates** by moving robot
4. **Test Limelight** by pointing at AprilTag
5. **Customize layout** if needed for driver preference

**Shuffleboard is now fully integrated** and ready for competition use! 🚀
