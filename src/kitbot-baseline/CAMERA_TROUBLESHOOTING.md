# Camera (/dev/video0) Connection Troubleshooting

## Problem
Limelight is detected but constantly tries to connect on `/dev/video0` and never connects successfully.

## Diagnostic Tools Added

### 1. Enhanced Camera Debugging
- **Startup**: Detailed camera initialization logging
- **Connection Attempts**: Specific error messages for each failure
- **Configuration**: Step-by-step camera setup verification

### 2. Camera Detection Command
- **Button**: Driver Controller **LEFT STICK** button
- **Function**: Scans all camera IDs (0-9) and reports availability
- **Output**: Comprehensive camera troubleshooting guide

## How to Diagnose

### Step 1: Run Camera Detection
1. Deploy updated code to robot
2. Press **LEFT STICK** on driver controller
3. Review the camera detection output

### Step 2: Check Console Messages
Watch for these specific messages during camera startup:
```
Attempting to start camera streaming...
Camera name: MainCamera
Camera ID: 0
Resolution: 320x240
FPS: 15
```

### Expected Results

#### If Camera Works:
```
Camera created successfully: MainCamera
Camera configured successfully
Started camera streaming: MainCamera (320x240 @ 15fps)
Stream URL: http://10.57.28.11:1181/stream.mjpg
```

#### If Camera Fails:
```
Failed to create camera with ID 0: [specific error]
This might be because:
1. No camera is connected to /dev/video0
2. The camera is in use by another application
3. The camera driver is not properly installed
4. The camera permissions are incorrect
```

## Common Issues and Solutions

### Issue: "No camera is connected to /dev/video0"
**Cause**: Limelight not properly detected as USB camera
**Solutions**:
- Check USB cable connection to Limelight
- Try different USB port on roboRIO
- Verify Limelight is powered (LED should be on)
- Restart roboRIO with Limelight connected

### Issue: "Camera is in use by another application"
**Cause**: Another process is using the camera
**Solutions**:
- Restart roboRIO to clear any stuck processes
- Check if vision software is running on roboRIO
- Disable any auto-start camera applications

### Issue: "Camera driver is not properly installed"
**Cause**: USB camera driver issues
**Solutions**:
- Update roboRIO system software
- Check if Limelight firmware is current
- Try different USB cable (some cables have power-only wires)

### Issue: "Camera permissions are incorrect"
**Cause**: Linux device permissions
**Solutions**:
- Restart roboRIO (usually fixes permission issues)
- Check if user has access to /dev/video0
- Contact FRC support if persistent

## Limelight-Specific Troubleshooting

### USB Mode Configuration
1. Access Limelight web interface (usually at `http://10.te.am.xx.xx:5801`)
2. Navigate to "Hardware" or "USB" settings
3. Ensure USB camera mode is enabled if using as camera
4. Check that USB streaming is configured

### NetworkTables vs USB Camera
- **Limelight Vision**: Uses NetworkTables (recommended)
- **Limelight as USB Camera**: Uses /dev/videoX (alternative)

**Recommendation**: Use NetworkTables approach for Limelight vision processing, not USB camera mode.

## Alternative Solutions

### Option 1: Change Camera ID
If detection shows camera available on different ID:
```java
// In CameraFeedStreamer constructor
private int cameraId = 1; // Change from 0 to detected ID
```

### Option 2: Use NetworkTables Instead
For Limelight, prefer the AprilTagSubsystem approach:
- Uses Limelight's built-in processing
- More reliable than USB camera mode
- Better performance

### Option 3: Disable Camera Streaming
If camera is not needed:
```java
// In RobotContainer constructor
// Comment out cameraStreamer.startStreaming();
```

## Testing the Fix

1. **Deploy code** to robot
2. **Press LEFT STICK** for camera detection
3. **Review output** - note which camera IDs are available
4. **Try camera controls**:
   - Driver POV Up: Enable streaming
   - Driver POV Down: Disable streaming
5. **Monitor console** for detailed connection messages

## Expected Behavior When Fixed

- Camera detection shows available camera(s)
- Camera initialization succeeds without errors
- Stream URL is accessible in browser
- No repeated connection attempts

## Technical Details

The camera system now includes:
- Detailed error reporting for each initialization step
- Camera availability scanning (IDs 0-9)
- Specific troubleshooting guidance for each error type
- Alternative configuration suggestions

Use these tools to identify the exact cause of the `/dev/video0` connection issue and apply the appropriate fix.
