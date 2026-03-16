# Limelight NetworkTables Setup Guide

## Overview
Switched from USB camera mode to NetworkTables-based Limelight vision processing. This is the recommended approach for FRC robots.

## What Changed

### ✅ Disabled USB Camera Streaming
- No more attempts to connect to `/dev/video0`
- CameraFeedStreamer is initialized but not started
- Removed camera control buttons (POV Up/Down, LT/RT triggers)

### ✅ Enhanced NetworkTables Integration
- AprilTagSubsystem provides comprehensive Limelight data
- SmartDashboard integration for real-time monitoring
- Automatic connection testing and status reporting

## Current Limelight Configuration

### NetworkTables Structure
```
limelight/
├── tv          (Valid target: 0 or 1)
├── tx          (Horizontal offset: -27 to 27 degrees)
├── ty          (Vertical offset: -20.5 to 20.5 degrees)
├── ta          (Target area: 0% to 100%)
├── tid         (Target ID: AprilTag number)
├── pipeline    (Current pipeline: 0-9)
└── camMode     (Camera mode: 0=vision, 1=driver)
```

### SmartDashboard Values
```
Limelight/
├── Connected           (Boolean: Limelight responding)
├── HasTarget          (Boolean: Target detected)
├── TargetID           (Number: AprilTag ID)
├── HorizontalOffset   (Number: tx value)
├── VerticalOffset     (Number: ty value)
├── TargetArea         (Number: ta value)
├── Distance           (Number: Distance in meters)
├── RobotX             (Number: Robot X position)
├── RobotY             (Number: Robot Y position)
├── RobotRotation      (Number: Robot rotation in degrees)
└── Environment        (String: COMPETITION/FOREIGN)
```

## Diagnostic Tools

### 1. Automatic Connection Testing
- **Startup**: Tests NetworkTables connection
- **Periodic**: Every 5 seconds during operation
- **Console**: Shows connection status and errors

### 2. Manual Diagnostic Command
- **Button**: Driver Controller **BACK** button
- **Function**: Comprehensive Limelight NetworkTables test
- **Output**: Connection status, target info, troubleshooting

### 3. Real-time Monitoring
- **SmartDashboard**: Live Limelight status and target data
- **Console**: Periodic connection updates
- **Error Messages**: Clear troubleshooting guidance

## Expected Behavior

### Normal Operation
```
=== LIMELIGHT CONNECTION TEST ===
Table path: limelight
tv (valid target): 0.0
tx (horizontal offset): 0.0
ty (vertical offset): 0.0
tid (target ID): -1.0
Limelight connection appears to be working
=================================
```

### When Target Detected
```
Limelight/Connected: true
Limelight/HasTarget: true
Limelight/TargetID: 4
Limelight/HorizontalOffset: 2.5
Limelight/VerticalOffset: -1.2
Limelight/Distance: 3.4
```

## Troubleshooting

### Connection Issues
**Symptoms**: `Limelight/Connected: false`
**Causes**: 
- Limelight not powered
- NetworkTables not working
- Wrong Limelight name
- Network issues

**Solutions**:
1. Check Limelight power (LED should be on)
2. Verify network connection
3. Access Limelight web interface
4. Check Limelight configuration

### Target Detection Issues
**Symptoms**: `Limelight/Connected: true` but `Limelight/HasTarget: false`
**Causes**:
- No AprilTags in view
- Wrong pipeline selected
- Poor lighting
- Limelight mode issues

**Solutions**:
1. Point Limelight at AprilTags
2. Check pipeline selection (0 for AprilTags)
3. Verify lighting conditions
4. Check Limelight web interface

## Limelight Web Interface

Access the Limelight configuration at:
```
http://10.te.am.xx.xx:5801
```

### Recommended Settings
- **Pipeline**: 0 (AprilTag detection)
- **Camera Mode**: Vision processing mode
- **NetworkTables**: Enabled
- **LED Mode**: Default or pipeline control

## Code Usage Examples

### Check for Target
```java
if (aprilTagSubsystem.hasTarget()) {
    int tagId = aprilTagSubsystem.getTargetId();
    double distance = aprilTagSubsystem.getDistanceToTarget();
    // Use target data...
}
```

### Get Robot Pose
```java
Optional<Pose2d> robotPose = aprilTagSubsystem.getRobotPose();
if (robotPose.isPresent()) {
    Pose2d pose = robotPose.get();
    // Use pose for localization...
}
```

### Target Alignment
```java
Optional<ChassisSpeeds> speeds = aprilTagSubsystem.getTargetSpeeds();
if (speeds.isPresent()) {
    ChassisSpeeds targetSpeeds = speeds.get();
    // Use speeds for alignment...
}
```

## Benefits of NetworkTables Approach

### ✅ More Reliable
- No USB driver issues
- No /dev/video0 problems
- Better error handling

### ✅ Better Performance
- Limelight does processing onboard
- Less CPU load on roboRIO
- Faster response times

### ✅ More Features
- Built-in AprilTag detection
- 3D pose estimation
- Multiple target support
- Distance calculations

### ✅ Easier Debugging
- Real-time status on SmartDashboard
- Comprehensive diagnostic tools
- Clear error messages

## Next Steps

1. **Deploy Code**: Upload updated code to robot
2. **Check Connection**: Press BACK button for diagnostic
3. **Monitor SmartDashboard**: Watch Limelight values
4. **Test with AprilTags**: Verify target detection
5. **Tune Settings**: Adjust pipeline if needed

The NetworkTables approach provides a much more robust and feature-rich Limelight integration compared to USB camera mode.
