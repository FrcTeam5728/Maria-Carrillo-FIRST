# NetworkTables Camera Setup Guide

## ✅ Pure NetworkTables Camera Integration

We've switched from complex CameraServer to pure NetworkTables approach for better reliability and simplicity.

## 🎯 What's Available:

### **📡 NetworkTables Camera System**
- **Pure NetworkTables** - No complex CameraServer dependencies
- **URL-based camera feeds** - Direct MJPEG stream URLs
- **Simple status monitoring** - Available/unavailable indicators
- **Shuffleboard integration** - Works with Camera widgets via URLs

## 🚀 How to Set Up Cameras in Shuffleboard:

### **Step 1: Launch Shuffleboard**
1. **Open Shuffleboard** on driver station
2. **Connect to robot** (Team 5728)
3. **Wait for NetworkTables** to populate (takes ~10 seconds)

### **Step 2: Add Limelight Camera**
1. **Click "+"** to add widget
2. **Select "Camera"** widget type
3. **Choose "Custom URL"** as source
4. **Enter URL:** `http://10.57.28.11:5800/stream.mjpg`
5. **Name it:** "Limelight"
6. **Resize** as needed

### **Step 3: Add USB Camera (Optional)**
1. **Connect USB camera** to robot
2. **Find camera stream URL** (depends on camera type)
3. **Add Camera widget** with custom URL
4. **Enter your USB camera URL**

### **Step 4: Monitor Camera Status**
1. **Go to Driver Tab** in Shuffleboard
2. **Look for camera status widgets:**
   - `Camera/USB_Available` - BooleanBox
   - `Camera/Limelight_Available` - BooleanBox
   - `Camera/Limelight_URL` - TextView

## 📊 NetworkTables Camera Data:

### **📡 Camera Table Structure:**
```
/Camera/
├── USB_Camera_URL (String)
├── Limelight_URL (String) 
├── USB_Camera_Available (Boolean)
└── Limelight_Available (Boolean)
```

### **📱 SmartDashboard Integration:**
```
Camera/
├── USB_Available (Boolean)
├── Limelight_Available (Boolean)
└── Limelight_URL (String)
```

## 🔧 Camera URL Formats:

### **🎯 Limelight (Always Available):**
```
http://10.57.28.11:5800/stream.mjpg
```

### **📷 USB Camera (Examples):**
```
# Standard USB cameras:
http://localhost:5800/stream.mjpg
http://10.57.28.11:5801/stream.mjpg

# IP cameras:
http://192.168.1.100/video.mjpg
http://10.57.28.12:8080/video
```

## 🛠️ Troubleshooting Camera Issues:

### **📷 Limelight Not Working:**
1. **Check NetworkTables:** Look for `/Camera/Limelight_Available = true`
2. **Test URL directly:** Open `http://10.57.28.11:5800/stream.mjpg` in browser
3. **Check network:** Ping `10.57.28.11` from driver station
4. **Check Limelight status:** LED should be on, connected to network

### **📱 Shuffleboard Issues:**
1. **NetworkTables not loading:** Wait 10-15 seconds after robot startup
2. **Camera widget black:** Check URL format and network connection
3. **Widget not updating:** Verify robot code is running

### **🔍 Debug Camera Connection:**
1. **Press START button** for Limelight test
2. **Check console output** for camera status messages
3. **Look for NetworkTables entries** in Shuffleboard
4. **Test URL in browser** to verify stream works

## 📋 Expected Console Output:

```
NetworkTablesCameraSubsystem initialized
Camera URLs published to NetworkTables
Use these URLs in Shuffleboard Camera widgets:
  USB: 
  Limelight: http://10.57.28.11:5800/stream.mjpg

Camera Status:
  USB Camera: NOT AVAILABLE
  Limelight: AVAILABLE
  Limelight URL: http://10.57.28.11:5800/stream.mjpg
  Add Camera widgets to Shuffleboard using NetworkTables entries
```

## 🎮 Controller Integration:

### **🔧 Camera Status Commands:**
- **Driver START Button:** Limelight connection test
- **Console Output:** Camera status every 10 seconds
- **SmartDashboard:** Real-time camera availability

## 📱 Shuffleboard Widget Setup:

### **📷 Camera Widget Configuration:**
```
Widget Type: Camera
Source: Custom URL
URL: http://10.57.28.11:5800/stream.mjpg
Name: Limelight
```

### **📊 Status Widget Configuration:**
```
Widget Type: BooleanBox
Source: /SmartDashboard/Camera/Limelight_Available
Name: Limelight Available
```

## ✅ What's Working Now:

- ✅ **Pure NetworkTables** camera system
- ✅ **Limelight URL** automatically published
- ✅ **Camera status** monitoring
- ✅ **Shuffleboard integration** via URLs
- ✅ **No CameraServer dependencies**
- ✅ **Simple and reliable** approach

## 🎯 Advantages of NetworkTables Approach:

1. **🔧 Simpler:** No complex CameraServer setup
2. **📡 More Reliable:** Pure NetworkTables, fewer dependencies
3. **🎮 Flexible:** Works with any camera that provides MJPEG stream
4. **📱 Easy Debugging:** URLs can be tested in browser
5. **⚡ Fast Startup:** No camera initialization delays

## 🔧 Advanced Configuration:

### **📷 Add Custom Camera URL:**
```java
// In NetworkTablesCameraSubsystem
cameraServer.setUsbCameraUrl("http://your-camera-url/stream.mjpg");
cameraServer.setUsbCameraAvailable(true);
```

### **📊 Monitor Camera Table:**
```java
// Access camera data directly
NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable("Camera");
String limelightUrl = cameraTable.getEntry("Limelight_URL").getString("");
boolean limelightAvailable = cameraTable.getEntry("Limelight_Available").getBoolean(false);
```

### **🔍 Test Camera Connection:**
```java
// Test Limelight connection
boolean connected = cameraServer.testLimelightConnection();
System.out.println("Limelight connected: " + connected);
```

## 🎯 Next Steps:

1. **Test Limelight** by adding camera widget with URL
2. **Verify NetworkTables** entries appear in Shuffleboard
3. **Test USB camera** if available
4. **Monitor status** widgets for connection health
5. **Customize URLs** for additional cameras if needed

**The NetworkTables camera system is now ready** and should work reliably with Shuffleboard! 🚀
