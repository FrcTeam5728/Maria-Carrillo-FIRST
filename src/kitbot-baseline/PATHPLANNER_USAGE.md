# PathPlanner Integration Usage Guide - Production Branch

## How to Make Your Differential Drive Robot Follow PathPlanner Paths

### 1. **Create Paths in PathPlanner GUI**
- Open PathPlanner application
- Load your project folder: `/home/physicsiscool314/TONK/src/kitbot-baseline`
- Create new paths by placing waypoints
- Save paths with names like "test_path", "score_path", etc.

### 2. **Autonomous Options Available**
Your robot now has these autonomous options on the dashboard:

- **PathPlanner Example**: Follows "example" path and shoots
- **PathPlanner Custom**: Follows "example" path + 2s shoot/1s intake pattern for 15s
- **Legacy Auto**: Original autonomous (still works)

### 3. **Using PathPlanner in Code**

#### **Follow a Single Path:**
```java
// Follow a named path
Command followPath = FollowPathPlannerPath.followNamedPath("my_path", driveSubsystem, true);
```

#### **Create Full Auto Routine:**
```java
// Auto from autos.json
Command auto = FollowPathPlannerPath.createAutoCommand("MyAuto");

// Custom sequence with path
Command customAuto = new SequentialCommandGroup(
    FollowPathPlannerPath.followNamedPath("approach", driveSubsystem, true),
    ballSubsystem.launchCommand().withTimeout(3),
    FollowPathPlannerPath.followNamedPath("retreat", driveSubsystem, false)
);
```

#### **PathPlanner AutoBuilder:**
```java
// Already configured in RobotContainer for differential drive
// Use AutoBuilder to create complex autos with multiple paths
```

### 4. **PathPlanner Files Structure**
```
src/main/deploy/pathplanner/
├── autos.json          # Autonomous routines
├── config.json          # Robot configuration
├── example.path         # Sample path
└── my_path.path         # Your custom paths
```

### 5. **Configuration Details**
- **Max Speed**: 3.0 m/s (configurable in FollowPathPlannerPath.java)
- **PID Gains**: Translation (5.0, 0.0, 0.0), Rotation (5.0, 0.0, 0.0)
- **Drive Type**: Differential drive (tank drive)
- **Auto-follows paths**: Field-relative driving with odometry reset

### 6. **Testing Your Path Following**
1. **Build and Deploy**: `./gradlew build deploy`
2. **Enable "PathPlanner Example"** in DriverStation autonomous selection
3. **Run Autonomous**: Robot should follow example path
4. **Check Console**: Look for "PathPlanner AutoBuilder configured" message

### 7. **Creating New Paths**
1. **Open PathPlanner GUI**
2. **Create waypoints** by clicking on the field
3. **Adjust constraints** (max velocity, acceleration)
4. **Save** with a descriptive name
5. **Use in code**: `FollowPathPlannerPath.followNamedPath("your_path_name", drive, true)`

### 8. **Troubleshooting**
- **Path not found**: Ensure .path file exists in `src/main/deploy/pathplanner/`
- **Robot doesn't move**: Check PID gains and max speed configuration
- **Odometry issues**: Verify gyro and encoders are working
- **Build errors**: Make sure PathPlannerLib vendor dependency is installed

### 9. **Differences from Swerve Branch**
- **Drive Type**: Differential drive (tank drive) instead of swerve
- **Path Following**: Uses `FollowPathCommand` instead of `FollowPathHolonomic`
- **Speed Control**: Converts chassis speeds to arcade drive inputs
- **No Strafing**: Cannot move sideways like swerve drive

### 10. **Advanced Usage**
- **Event Markers**: Add actions at specific points along paths
- **Path Groups**: Combine multiple paths in sequence
- **Dynamic Replanning**: Auto-replan if robot deviates from path

Your differential drive robot is now fully integrated with PathPlanner! 🚀
