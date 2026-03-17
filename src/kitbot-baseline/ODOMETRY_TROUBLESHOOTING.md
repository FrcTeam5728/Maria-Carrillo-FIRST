# Odometry Troubleshooting Guide

## Why Odometry Might Not Be Working

Based on the code analysis, here are the most likely reasons odometry isn't working:

## ✅ Current Odometry System Status

### What We Have:
- **DriveSubsystem**: Built-in odometry with encoders and gyro
- **VisionOdometry**: Advanced vision fusion system (NOT being used)
- **FieldPositionSystem**: NetworkTables position publisher
- **LimelightSubsystem**: Vision targeting system

### What's Actually Working:
- **DriveSubsystem Odometry**: ✅ Has encoders, gyro, and periodic updates
- **FieldPositionSystem**: ✅ Publishes position to NetworkTables
- **LimelightSubsystem**: ✅ Provides target data

### What's NOT Working:
- **VisionOdometry**: ❌ Created but never instantiated or updated

## 🔍 Diagnostic Tools

### New Diagnostic Command:
**Driver Controller RIGHT BUMPER**: Odometry diagnostic
- Shows DriveSubsystem odometry data
- Shows Limelight target data
- Shows FieldPositionSystem data
- Provides troubleshooting suggestions

### SmartDashboard Values:
```
Drive/Odometry/X: Current X position
Drive/Odometry/Y: Current Y position
Drive/Odometry/Heading: Current heading
Drive/Gyro/Angle: Gyro angle
Drive/Encoder/Left: Left encoder distance
Drive/Encoder/Right: Right encoder distance
```

## 🚨 Common Odometry Issues

### 1. Robot Not Moving (Position Stays at 0,0)
**Symptoms:**
- Odometry always shows (0.0, 0.0)
- Encoder values not changing
- Robot appears stationary in odometry

**Causes:**
- Robot not actually moving (check motors)
- Encoder connections broken
- Encoder initialization failed
- Gyro not connected

**Solutions:**
1. Press **RIGHT BUMPER** for diagnostic
2. Check if robot physically moves
3. Verify encoder wiring
4. Check SmartDashboard encoder values

### 2. Odometry Drifting or Inaccurate
**Symptoms:**
- Position slowly drifts over time
- Heading doesn't match actual robot direction
- Position jumps unexpectedly

**Causes:**
- Gyro not calibrated
- Encoder distance per pulse wrong
- Wheel slippage
- Mechanical issues

**Solutions:**
1. Calibrate gyro (automatic on init)
2. Check encoder distance per pulse settings
3. Verify wheel radius constant
4. Check for wheel slippage

### 3. Vision Odometry Not Working
**Symptoms:**
- Vision corrections not applied
- Limelight data not used
- Position doesn't update with AprilTags

**Causes:**
- VisionOdometry class not instantiated
- VisionOdometry.update() never called
- Limelight not connected
- No AprilTag targets visible

**Solutions:**
1. VisionOdometry needs to be integrated into RobotContainer
2. Add VisionOdometry.update() to periodic calls
3. Check Limelight connection with START button
4. Point Limelight at AprilTags

## 🔧 Step-by-Step Troubleshooting

### Step 1: Basic Odometry Check
1. **Press RIGHT BUMPER** (driver controller)
2. Move robot manually
3. Watch DriveSubsystem odometry values
4. Check if position changes

### Step 2: Sensor Status Check
1. Look at SmartDashboard values
2. Check Drive/Gyro/Angle changes when robot rotates
3. Check Drive/Encoder/Left and Right values when robot moves
4. Verify all sensors are updating

### Step 3: Limelight Integration
1. **Press START button** for Limelight test
2. Point Limelight at AprilTag
3. Verify target detection
4. Check distance and offset values

### Step 4: Field Position System
1. Check Field/RobotX and Field/RobotY on SmartDashboard
2. Verify confidence levels
3. Check position source (VISION/ENCODERS)
4. Compare with DriveSubsystem odometry

## 📊 Expected Behavior

### Working Odometry Should Show:
```
Drive/Odometry/X: Changes when robot moves forward/backward
Drive/Odometry/Y: Changes when robot moves left/right
Drive/Odometry/Heading: Changes when robot rotates
Drive/Gyro/Angle: Matches robot rotation
Drive/Encoder/Left: Increases when left wheel moves forward
Drive/Encoder/Right: Increases when right wheel moves forward
```

### Field Position System Should Show:
```
Field/RobotX: Similar to Drive/Odometry/X
Field/RobotY: Similar to Drive/Odometry/Y
Field/Confidence: High when Limelight has target
Field/PositionSource: "VISION" when target detected
```

## 🛠️ Advanced Troubleshooting

### Check DriveSubsystem Implementation:
The DriveSubsystem has complete odometry support:
- Encoders with proper distance per pulse
- Gyro with calibration
- Periodic updates in periodic() method
- SmartDashboard publishing

### Common Code Issues:
```java
// This works (DriveSubsystem has built-in odometry)
var pose = driveSubsystem.getPose();

// This doesn't work (VisionOdometry not instantiated)
VisionOdometry visionOdometry = new VisionOdometry(driveSubsystem, limelightSubsystem);
visionOdometry.update(); // Never called!
```

### Sensor Initialization:
DriveSubsystem automatically initializes:
- Encoders in initializeEncoders()
- Gyro in initializeGyro()
- Odometry in initializeOdometry()

## 🎯 Quick Fixes

### If Odometry Shows (0,0,0):
1. Check if robot actually moves
2. Verify encoder connections
3. Check motor controller configuration
4. Press RIGHT BUMPER for diagnostic

### If Odometry Drifts:
1. Recalibrate gyro (restart robot)
2. Check encoder distance per pulse
3. Verify wheel radius constant
4. Reduce wheel slippage

### If Vision Not Working:
1. VisionOdometry class needs integration
2. Check Limelight connection
3. Point at AprilTags
4. Verify target detection

## 📋 Diagnostic Checklist

- [ ] Robot physically moves when commanded
- [ ] Drive/Odometry values change with movement
- [ ] Drive/Gyro/Angle changes with rotation
- [ ] Drive/Encoder values change with wheel movement
- [ ] Limelight connected (START button test)
- [ ] AprilTag detected when pointed at target
- [ ] Field/Position updates with movement
- [ ] Position confidence reasonable (>0.3)

## 🔮 Next Steps

To fully fix odometry:
1. **Integrate VisionOdometry** into RobotContainer
2. **Add VisionOdometry.update()** to periodic calls
3. **Test sensor fusion** with actual robot movement
4. **Tune confidence thresholds** for vision updates
5. **Add odometry reset** functionality

The DriveSubsystem odometry should work out of the box. The VisionOdometry system exists but needs integration to provide sensor fusion capabilities.
