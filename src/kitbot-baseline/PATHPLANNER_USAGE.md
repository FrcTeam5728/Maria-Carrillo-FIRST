# PathPlanner Integration Usage Guide

## How to Make Your Robot Follow PathPlanner Paths

### 1. **Create Paths in PathPlanner GUI**
- Open PathPlanner application
- Load your project folder: `/home/physicsiscool314/TONK/src/kitbot-baseline`
- Create new paths by placing waypoints
- Save paths with names like "test_path", "score_path", etc.

### 2. **Autonomous Options Available**
Your robot now has these autonomous options on the dashboard:

- **PathPlanner Example**: Follows the "example" path and shoots
- **PathPlanner Custom**: Follows "example" path + 2s shoot/1s intake pattern for 15s
- **Swerve Auto**: Demonstrates swerve capabilities (strafing, field-relative driving)
- **Quick Swerve**: Simple forward movement with shooting
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
// Already configured in RobotContainer for swerve drive
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
- **Max Speed**: 4.5 m/s (configurable in FollowPathPlannerPath.java)
- **PID Gains**: Translation (5.0, 0.0, 0.0), Rotation (5.0, 0.0, 0.0)
- **Drive Base Radius**: 0.4 m
- **Auto-follows paths**: Field-relative driving with odometry reset

### 6. **Testing Your Path Following**
1. **Build and Deploy**: `./gradlew build deploy`
2. **Enable "PathPlanner Example"** in DriverStation autonomous selection
3. **Run Autonomous**: Robot should follow the example path
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

### 9. **Advanced Usage**
- **Event Markers**: Add actions at specific points along paths
- **Choreo Trajectories**: Use pre-generated trajectories
- **Path Groups**: Combine multiple paths in sequence
- **Dynamic Replanning**: Auto-replan if robot deviates from path

Your swerve robot is now fully integrated with PathPlanner! 🚀
