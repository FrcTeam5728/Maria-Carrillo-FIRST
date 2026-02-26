# FRC Robot System Dictionary

*Comprehensive guide for maintaining and extending the robot codebase after the original developers leave.*

---

## Table of Contents
1. [Architecture Overview](#architecture-overview)
2. [Core Systems](#core-systems)
3. [Vision System](#vision-system)
4. [Path Planning](#path-planning)
5. [Drive Systems](#drive-systems)
6. [Constants and Configuration](#constants-and-configuration)
7. [Commands and Control](#commands-and-control)
8. [Factory Pattern](#factory-pattern)
9. [Build and Deployment](#build-and-deployment)
10. [Troubleshooting](#troubleshooting)

---

## Architecture Overview

### System Philosophy
- **Modular Design**: Each major system is encapsulated in its own subsystem
- **Factory Pattern**: Subsystem creation is centralized in `RobotSubsystemFactory`
- **Constraint-Based Vision**: Vision tracking uses a flexible constraint system
- **Vendor Abstraction**: Supports multiple motor controller types (SparkMax, VictorSPX)

### Key Design Patterns
- **Command-Based Programming**: Uses WPILib command framework
- **Subsystem Pattern**: Each physical system maps to a software subsystem
- **Factory Pattern**: Centralized subsystem creation and configuration
- **Constraint Pattern**: Vision behaviors are modular and composable

---

## Core Systems

### RobotContainer
**Location**: `src/main/java/frc/robot/RobotContainer.java`

**Purpose**: Central hub for robot initialization, button bindings, and autonomous selection.

**Key Responsibilities**:
- Creates all subsystems via `RobotSubsystemFactory`
- Configures driver/operator controller bindings
- Sets up autonomous command chooser
- Provides default commands for subsystems

**Important Notes**:
- Uses `TeleopControlModifier` to apply vision constraints to driver input
- PathPlanner integration for autonomous routines
- Controller ports defined in `OperatorConstants`

---

## Vision System

### AprilTagSubsystem
**Location**: `src/main/java/frc/robot/subsystems/vision/AprilTagSubsystem.java`

**Purpose**: Handles AprilTag detection and tracking using Limelight cameras.

**Key Features**:
- **Limelight Integration**: Reads from NetworkTable for target data
- **Constraint System**: Modular behaviors for distance, rotation, and positioning
- **Pose Estimation**: Calculates robot pose from AprilTag detections
- **Speed Calculation**: Provides chassis speeds for automatic targeting

**Important Methods**:
- `hasTarget()`: Check if AprilTag is detected
- `getTargetSpeeds()`: Get constrained movement speeds
- `getForwardSpeed()` / `getRotationSpeed()`: Individual speed components
- `getRobotPose()`: Field-relative pose estimation

**Configuration Constants**:
```java
// In AprilTagSubsystem constructor
MOUNT_ANGLE_DEGREES = 0.0;    // Camera mounting angle
LENS_HEIGHT_METERS = 0.0;     // Camera height from floor
TARGET_HEIGHT_METERS = 0.0;   // AprilTag height from floor
```

### Constraint System
**Location**: `src/main/java/frc/robot/subsystems/vision/constraints/`

**Purpose**: Modular vision behaviors that can be combined and configured.

#### AprilTagConstraint (Interface)
Base interface for all vision constraints.

#### DistanceConstraint
Maintains specific distance from AprilTag.
- **PID Control**: Uses PID controller for distance regulation
- **Configurable**: Target distance and PID gains adjustable
- **Methods**: `setTargetDistance()`, `setSetpoint()`

#### RotationConstraint
Keeps robot facing the AprilTag.
- **Yaw Control**: Aligns robot horizontally with target
- **Continuous Input**: Handles angle wrapping (-180° to 180°)
- **Tolerance**: 2-degree default tolerance

#### PositionConstraint
Controls lateral positioning relative to AprilTag.
- **2D Control**: Manages X/Y offsets from target
- **Future Feature**: Currently implemented but not actively used

#### ConstraintManager
Manages active constraints and combines their outputs.
- **Dynamic**: Constraints can be enabled/disabled at runtime
- **Combinatorial**: Multiple constraints can work simultaneously

### LimelightTarget
**Location**: `src/main/java/frc/robot/subsystems/vision/limelight/LimelightTarget.java`

**Purpose**: Data object representing a detected AprilTag.

**Key Properties**:
- `tx`: Horizontal offset from crosshair (-27° to 27°)
- `ty`: Vertical offset from crosshair (-20.5° to 20.5°)
- `ta`: Target area (0% to 100% of image)
- `tid`: AprilTag ID
- `distanceMeters`: Calculated distance to target

**Helper Methods**:
- `getYawDegrees()`: Horizontal angle (same as tx)
- `getX()` / `getY()`: Estimated position relative to target

---

## Path Planning

### PathPlannerSubsystem
**Location**: `src/main/java/frc/robot/subsystems/PathPlannerSubsystem.java`

**Purpose**: Autonomous path following using PathPlanner library.

**Key Features**:
- **PathPlanner Integration**: Loads paths from PathPlanner GUI
- **Ramsete Control**: Uses proven path following algorithm
- **Differential Drive**: Optimized for tank drive robots
- **Voltage Control**: Direct voltage output for precise control

**Important Methods**:
- `followPath(String pathName)`: Creates command to follow named path
- `getPose()`: Current robot pose from odometry
- `getCurrentSpeeds()`: Current chassis speeds
- `drive(ChassisSpeeds)`: Low-level chassis control

**Path Following Process**:
1. Load path from PathPlanner using `PathPlannerPath.fromPathFile()`
2. Convert to WPILib trajectory
3. Create RamseteCommand with proper feedforward
4. Apply voltage constraints to prevent wheel slip
5. Execute path with automatic stopping

**Constants Used**:
- `kMaxSpeedMetersPerSecond`: Maximum robot speed
- `kMaxAccelerationMetersPerSecondSquared`: Maximum acceleration
- `ksVolts`, `kvVoltSecondsPerMeter`, `kaVoltSecondsSquaredPerMeter`: Feedforward gains

---

## Drive Systems

### DriveSubsystem (Abstract)
**Location**: `src/main/java/frc/robot/subsystems/DriveSubsystem.java`

**Purpose**: Abstract base class for all drive implementations.

**Key Features**:
- **Common Interface**: Provides consistent API across motor types
- **Differential Drive**: Built-in WPILib DifferentialDrive support
- **Odometry**: Pose tracking and reset capabilities

**Abstract Methods** (must implement):
- `tankDriveVolts(double, double)`: Voltage-based tank drive
- `getWheelSpeeds()`: Current wheel speed measurement

**Provided Methods**:
- `arcadeDrive(double, double)`: Arcade drive control
- `driveArcade(DoubleSupplier, DoubleSupplier)`: Command-based arcade drive
- `stop()`: Emergency stop
- `getPose()` / `resetOdometry()`: Pose management

### DriveSubsystemSparkMax
**Location**: `src/main/java/frc/robot/subsystems/DriveSubsystemSparkMax.java`

**Purpose**: SparkMax-based drive implementation.

**Key Features**:
- **Brushed Motors**: Configured for CIM-style brushed motors
- **Encoder Support**: Built-in encoder feedback for wheel speeds
- **Voltage Compensation**: Consistent performance across battery voltage
- **Current Limiting**: Prevents breaker trips

**Configuration**:
- Motor IDs from `DriveConstants.LEFT_LEADER_ID_SPARKMAX`, etc.
- Current limit: `DRIVE_MOTOR_CURRENT_LIMIT`
- Voltage compensation: 12V

### DriveSubsystemVictorSpx
**Location**: `src/main/java/frc/robot/subsystems/DriveSubsystemVictorSpx.java`

**Purpose**: VictorSPX-based drive implementation.

**Key Features**:
- **CTRE Integration**: Uses Phoenix API for VictorSPX
- **Voltage Compensation**: Maintains consistent speed
- **No Encoders**: Returns zero wheel speeds (external encoders needed for odometry)

**Limitations**:
- No built-in encoder support
- Wheel speed estimation requires external sensors

---

## Constants and Configuration

### Constants.java
**Location**: `src/main/java/frc/robot/Constants.java`

**Purpose**: Centralized configuration for all robot parameters.

### DriveConstants
```java
// Motor Controller IDs
LEFT_LEADER_ID_SPARKMAX = 21
LEFT_FOLLOWER_ID_SPARKMAX = 22
RIGHT_LEADER_ID_SPARKMAX = 23
RIGHT_FOLLOWER_ID_SPARKMAX = 24

// Physical Properties
kWheelDiameterMeters = 0.1524     // 6 inches
kGearRatio = 10.71                  // Gear reduction
kTrackWidthMeters = 0.69           // 27.2 inches

// Performance Limits
kMaxSpeedMetersPerSecond = 3.0
kMaxAccelerationMetersPerSecondSquared = 3.0
kMaxAngularSpeedRadiansPerSecond = Math.PI

// Feedforward Gains (for path following)
ksVolts = 0.1                       // Static voltage
kvVoltSecondsPerMeter = 1.5         // Velocity gain
kaVoltSecondsSquaredPerMeter = 0.2  // Acceleration gain

// PID Gains
kPDriveVel = 8.5
kIDriveVel = 0.0
kDDriveVel = 0.0
```

### FactoryConstants
```java
DRIVE_SUBSYSTEM_TYPE = "SPARKMAX"  // or "VICTORSPX"
FUEL_SUBSYSTEM_TYPE = "SPARKMAX"    // or "VICTORSPX"
```

### OperatorConstants
```java
DRIVER_CONTROLLER_PORT = 0
OPERATOR_CONTROLLER_PORT = 0        // Same controller for both
DRIVE_SCALING = 0.7                 // Speed reduction factor
ROTATION_SCALING = 0.8              // Rotation reduction factor
```

### FuelConstants
Motor IDs and voltage values for fuel mechanism (intake/launcher).

---

## Commands and Control

### AprilTagTrackCommand
**Location**: `src/main/java/frc/robot/commands/AprilTagTrackCommand.java`

**Purpose**: Combines driver input with automatic AprilTag tracking.

**Behavior**:
- Driver provides forward/rotation input via joysticks
- When AprilTag detected, automatically adjusts movement
- Blends manual and automatic control (50% blend factor)
- Respects speed limits and applies rate limiting

**Usage**:
```java
new AprilTagTrackCommand(
    aprilTagSubsystem,
    driveSubsystem,
    () -> -driverController.getLeftY() * DRIVE_SCALING,
    () -> -driverController.getRightX() * ROTATION_SCALING,
    maxSpeed
)
```

### TeleopControlModifier
**Location**: `src/main/java/frc/robot/utils/TeleopControlModifier.java`

**Purpose**: Applies vision constraints to driver input.

**Features**:
- **Deadband**: Removes small joystick movements
- **Constraint Blending**: Mixes driver input with vision corrections
- **Rate Limiting**: Prevents sudden control changes
- **Enable/Disable**: Constraints can be toggled at runtime

**Key Method**:
```java
double[] applyConstraints(double forward, double rotation)
// Returns: [modifiedForward, modifiedRotation]
```

---

## Factory Pattern

### RobotSubsystemFactory
**Location**: `src/main/java/frc/robot/RobotSubsystemFactory.java`

**Purpose**: Centralized creation and configuration of all subsystems.

**Benefits**:
- **Single Point of Change**: Modify subsystem types in one place
- **Configuration Management**: All subsystem setup in one location
- **Testing**: Easy to mock subsystems for unit tests

**Key Methods**:
```java
createDriveSubsystem()        // Creates based on DRIVE_SUBSYSTEM_TYPE
createFuelSubsystem()         // Creates based on FUEL_SUBSYSTEM_TYPE
createAprilTagSubsystem()     // Always creates AprilTagSubsystem
```

**Adding New Subsystems**:
1. Add creation method to `RobotSubsystemFactory`
2. Add configuration constants to appropriate Constants class
3. Wire up in `RobotContainer`

---

## Build and Deployment

### Build System
- **GradleRIO**: Standard FRC build system
- **Java 17**: Required Java version
- **WPILib 2026.2.1**: WPILib framework version

### Vendor Dependencies
**Location**: `vendordeps/`

**Key Dependencies**:
- `PathplannerLib-2026.1.2.json`: Path planning and following
- `Phoenix5-replay-5.36.0.json`: CTRE motor controllers
- `Phoenix6-replay-26.1.1.json`: New CTRE API
- `REVLib.json`: REV Robotics motor controllers
- `ReduxLib-2026.1.1.json`: Additional utilities

### Building and Deploying
```bash
# Build the project
./gradlew build

# Deploy to robot
./gradlew deploy

# Run simulation
./gradlew simulate
```

---

## Troubleshooting

### Common Issues

#### PathPlanner Errors
- **Symptom**: "FollowPathCommand constructor undefined"
- **Cause**: API mismatch between PathPlanner versions
- **Solution**: Use RamseteCommand approach as shown in PathPlannerSubsystem

#### Vision System Not Working
- **Check**: Limelight network table connection
- **Verify**: Camera mounting angles in AprilTagSubsystem
- **Test**: Target detection with Limelight web interface

#### Drive System Issues
- **SparkMax**: Check CAN IDs and motor wiring
- **VictorSPX**: Verify Phoenix API version compatibility
- **Encoders**: Ensure encoder initialization in drive subsystem

#### Constraint System Problems
- **Symptom**: Robot not responding to AprilTag
- **Check**: Constraint activation states
- **Verify**: PID constants are reasonable
- **Test**: Individual constraints in isolation

### Debugging Tools

#### NetworkTables
- **Limelight**: Check `limelight` table for target data
- **SmartDashboard**: Add telemetry for debugging

#### Log Output
- **DriverStation**: Check for error messages
- **Console**: Use `System.out.println()` for debugging

#### Simulation
- **Use**: WPILib simulation for testing without robot
- **Benefits**: Faster iteration, safe testing

---

## Adding New Features

### New Subsystem
1. Create subsystem class extending appropriate base
2. Add constants to Constants.java
3. Add creation method to RobotSubsystemFactory
4. Wire up in RobotContainer
5. Add commands as needed

### New Vision Constraint
1. Implement AprilTagConstraint interface
2. Add to ConstraintManager in AprilTagSubsystem
3. Configure PID constants and behavior
4. Test with existing constraint system

### New Autonomous Routine
1. Create path in PathPlanner GUI
2. Add to auto chooser in RobotContainer
3. Test in simulation before deploying

---

## Performance Tuning

### Path Following
- **Ramsete Gains**: Adjust `b` and `zeta` parameters in RamseteController
- **Feedforward**: Tune `ks`, `kv`, `ka` for your robot
- **Constraints**: Adjust voltage and acceleration limits

### Vision Tracking
- **PID Constants**: Tune each constraint's PID gains
- **Blend Factor**: Adjust automatic vs manual control mixing
- **Rate Limiting**: Prevent jerky movements

### Drive Performance
- **Current Limits**: Adjust for motor protection vs performance
- **Voltage Compensation**: Ensure consistent behavior
- **Gear Ratio**: Verify physical gearing matches code

---

## Safety Considerations

### Motor Safety
- **Watchdog**: WPILib motor safety enabled by default
- **Current Limits**: Prevent motor damage and breaker trips
- **Voltage Limits**: Protect electrical system

### Vision Safety
- **Target Validation**: Verify target ID before acting
- **Fallback Behavior**: Return to manual control if vision lost
- **Speed Limits**: Enforce maximum speeds in all modes

### Autonomous Safety
- **Path Validation**: Ensure paths are within field boundaries
- **Emergency Stop**: Always provide manual override capability
- **Timeout Protection**: Commands should have reasonable timeouts

---

## Contact and Resources

### Documentation
- **WPILib Docs**: https://docs.wpilib.org/
- **PathPlanner Docs**: https://pathplanner.lib/
- **Limelight Docs**: https://docs.limelightvision.io/

### Community
- **FRC Discord**: Active community support
- **Chief Delphi**: Technical discussions
- **GitHub Issues**: Report bugs and request features

### Vendor Documentation
- **REV Robotics**: SparkMax and motor controllers
- **CTRE**: VictorSPX and TalonFX documentation
- **Limelight**: Camera setup and configuration

---

*This dictionary should be updated whenever significant changes are made to the robot codebase. Please keep it current for future maintainers.*
