# Field Widget Usage Guide

## 🎯 Field Visualization in Shuffleboard

Your robot now has a **Field2d widget** in Shuffleboard that shows the robot's current position on the field!

### **What's Available:**

1. **Visual Field Display** - Shows robot position and heading on a field diagram
2. **Real-time Updates** - Updates automatically as robot moves
3. **Path Visualization** - Can display PathPlanner paths and trajectories
4. **Position Data** - Numerical X, Y, and heading values

### **Location in Shuffleboard:**

- **Tab**: Driver
- **Position**: Top center (4x3 grid space)
- **Widget**: Field2d with robot visualization

### **Features:**

#### **🤖 Robot Position**
- **Blue Arrow**: Robot position and heading
- **Real-time**: Updates every 20ms
- **Coordinate System**: Field-relative coordinates

#### **📍 Position Values**
- **Robot X**: Horizontal position (meters)
- **Robot Y**: Vertical position (meters) 
- **Robot Heading**: Gyro angle (degrees)
- **Gyro Angle**: Raw gyro reading

#### **🛤️ Path Visualization**
You can display paths on the field using these methods:

```java
// Display a PathPlanner trajectory
shuffleboardManager.displayTrajectory(trajectory, "Auto Path");

// Display custom path from poses
Pose2d[] path = {startPose, midPose, endPose};
shuffleboardManager.displayPath(path, "Custom Path");

// Clear all paths
shuffleboardManager.clearTrajectories();
```

### **Usage Examples:**

#### **1. Basic Position Monitoring**
- Open Shuffleboard
- Go to Driver tab
- Watch the field widget as you drive the robot
- See position values update in real-time

#### **2. PathPlanner Visualization**
```java
// In your autonomous command
@Override
public void initialize() {
    var trajectory = PathPlannerPath.fromPathFile("my_path").getTrajectory();
    RobotContainer.getInstance().getShuffleboardManager()
        .displayTrajectory(trajectory, "Current Path");
}
```

#### **3. Debugging Odometry**
- Compare field widget position with actual robot position
- Verify odometry is working correctly
- Check for drift or positioning errors

### **Field Coordinates:**

- **Origin**: Bottom-left corner of field
- **X Axis**: Horizontal (left to right)
- **Y Axis**: Vertical (bottom to top)
- **Units**: Meters
- **Heading**: 0° = facing positive X direction

### **Troubleshooting:**

#### **Robot Not Visible:**
- Check if odometry is initialized
- Verify gyro and encoders are working
- Ensure `updateOdometryWidgets()` is being called

#### **Position Incorrect:**
- Reset odometry: `driveSubsystem.resetOdometry(new Pose2d())`
- Check encoder values in debug tab
- Verify gyro calibration

#### **Path Not Displaying:**
- Ensure trajectory is not null
- Check path name is valid
- Use `clearTrajectories()` before adding new paths

### **Integration with PathPlanner:**

The field widget is perfect for visualizing PathPlanner paths:

```java
// In FollowPathPlannerPath command
@Override
public void initialize() {
    // Display the path on the field
    var trajectory = m_path.getTrajectory();
    RobotContainer.getInstance().getShuffleboardManager()
        .displayTrajectory(trajectory, "Following Path");
}

@Override
public void end(boolean interrupted) {
    // Clear the path when done
    RobotContainer.getInstance().getShuffleboardManager()
        .clearTrajectories();
}
```

### **Advanced Usage:**

#### **Multiple Paths:**
```java
// Display different colored paths
shuffleboardManager.displayTrajectory(path1, "Path 1");
shuffleboardManager.displayTrajectory(path2, "Path 2");
shuffleboardManager.displayTrajectory(path3, "Path 3");
```

#### **Custom Field Objects:**
```java
// Add custom objects to the field
fieldWidget.getObject("Target").setPose(targetPose);
fieldWidget.getObject("Obstacle").setPose(obstaclePose);
```

Your field widget is now ready for use! 🚀

**Next Steps:**
1. Deploy and test the robot
2. Open Shuffleboard Driver tab
3. Drive the robot and watch the position update
4. Try displaying PathPlanner paths when available
