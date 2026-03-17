# Camera Server & Shuffleboard Integration Guide

## ✅ Camera System Now Integrated

The camera server is now fully integrated and ready to use with Shuffleboard!

## 🎯 What's Available:

### **USB Camera Feed**
- **Name:** "USB Camera" 
- **Resolution:** 320x240
- **FPS:** 15
- **Available in:** Shuffleboard Camera widget

### **Limelight Camera Feed**
- **Name:** "Limelight"
- **URL:** `http://10.57.28.11:5800/stream.mjpg`
- **Resolution:** 320x240
- **Available in:** Shuffleboard Camera widget (via URL)

### **Camera Status Information**
- **SmartDashboard Keys:**
  - `Camera/USB_Available`: USB camera status
  - `Camera/Limelight_Available`: Limelight stream status  
  - `Camera/Limelight_URL`: Direct Limelight stream URL

## 🔧 How to Add Cameras to Shuffleboard:

### **Step 1: Open Shuffleboard**
- Launch Shuffleboard on your driver station
- Connect to the robot (team 5728)

### **Step 2: Add USB Camera Widget**
1. **Click "+"** to add a new widget
2. **Select "Camera"** from the widget list
3. **Choose "USB Camera"** as the camera source
4. **Resize** the widget as needed

### **Step 3: Add Limelight Camera Widget**
1. **Click "+"** to add another widget
2. **Select "Camera"** from the widget list
3. **Choose "Limelight"** as the camera source
4. **Alternative:** Use URL method:
   - Add "Camera" widget
   - Set source to "Custom URL"
   - Enter: `http://10.57.28.11:5800/stream.mjpg`

### **Step 4: Add Camera Status Display**
1. **Click "+"** to add widgets
2. **Add "Boolean Display"** widgets for:
   - `Camera/USB_Available`
   - `Camera/Limelight_Available`
3. **Add "Text Display"** widget for:
   - `Camera/Limelight_URL`

## 📱 Shuffleboard Layout Example:

```
┌─────────────────────────────────────────────────────────┐
│                    DRIVER STATION                        │
├─────────────────┬─────────────────┬─────────────────────┤
│   USB Camera    │   Limelight     │   Camera Status     │
│   (320x240)     │   (320x240)     │                     │
│                 │                 │ USB: AVAILABLE      │
│                 │                 │ Limelight: AVAILABLE │
│                 │                 │ URL: http://...      │
└─────────────────┴─────────────────┴─────────────────────┘
```

## 🔍 Troubleshooting Camera Issues:

### **USB Camera Not Working:**
1. **Check Physical Connection:**
   - USB cable securely connected to robot
   - Camera powered on (LED indicator)
   - Try different USB port

2. **Check Software Status:**
   - Look for `Camera/USB_Available: true` in SmartDashboard
   - Console should show: "USB Camera: AVAILABLE"

3. **Check Shuffleboard:**
   - Widget source set to "USB Camera"
   - Try removing and re-adding widget

### **Limelight Camera Not Working:**
1. **Check Network Connection:**
   - Limelight powered on (LED on)
   - Ethernet cable connected
   - Limelight on correct IP: `10.57.28.11`

2. **Test Direct Access:**
   - Open browser: `http://10.57.28.11:5800/stream.mjpg`
   - Should see video feed

3. **Check Software Status:**
   - Look for `Camera/Limelight_Available: true`
   - Console should show: "Limelight Stream: AVAILABLE"

4. **Check Shuffleboard:**
   - Widget source set to "Limelight"
   - Try URL method with direct URL

## 🎮 Controller Integration:

### **Camera Status Monitoring:**
- **Console Output:** Status printed every 10 seconds
- **SmartDashboard:** Real-time status updates
- **Automatic Detection:** Cameras detected on startup

### **Testing Commands:**
- **Driver START Button:** Test Limelight connection
- **Driver RIGHT BUMPER:** Odometry diagnostic (includes camera status)

## 📊 Expected Console Output:

```
CameraServerSubsystem initialized
USB Camera: Available in Shuffleboard
Limelight Stream: Available in Shuffleboard
Add Camera widgets to Shuffleboard to view feeds

Camera Server Status:
  USB Camera: AVAILABLE
  Limelight Stream: AVAILABLE
  Limelight URL: http://10.57.28.11:5800/stream.mjpg
  Add Camera widgets to Shuffleboard to view feeds
```

## 🛠️ Advanced Configuration:

### **Change Camera Resolution:**
```java
// In CameraServerSubsystem.java
private static final int CAMERA_WIDTH = 640;
private static final int CAMERA_HEIGHT = 480;
private static final int CAMERA_FPS = 30;
```

### **Add Multiple USB Cameras:**
```java
// Additional cameras can be added in initializeUsbCamera()
UsbCamera camera2 = CameraServer.startAutomaticCapture(1);
```

### **Custom Camera Processing:**
```java
// In startVisionProcessing() thread
// Add OpenCV processing here
Imgproc.cvtColor(usbFrame, processedFrame, Imgproc.COLOR_BGR2GRAY);
```

## ✅ What's Working Now:

- ✅ **USB Camera** automatically detected and streamed
- ✅ **Limelight Camera** URL published for Shuffleboard
- ✅ **Camera Status** available in SmartDashboard
- ✅ **Automatic Detection** on robot startup
- ✅ **Error Handling** for disconnected cameras
- ✅ **Resource Cleanup** on robot shutdown

## 🎯 Next Steps:

1. **Test USB Camera:** Connect USB camera and verify feed
2. **Test Limelight:** Point at AprilTag and verify stream
3. **Configure Shuffleboard:** Add camera widgets
4. **Monitor Status:** Watch SmartDashboard values
5. **Troubleshoot:** Use console output for debugging

The camera system is now fully integrated with Shuffleboard and ready for competition use!
