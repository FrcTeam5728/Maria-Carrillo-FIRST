# NetworkTables Field Position System

## Overview
Created a comprehensive NetworkTables-based field position system that shows where the robot thinks it is on the field in real-time. This system publishes position data that can be visualized using tools like Shuffleboard, Glass, or custom dashboards.

## ✅ Features

**Real-Time Position Tracking:**
- Robot X, Y coordinates in meters
- Robot rotation in degrees
- Position confidence level (0.0 to 1.0)
- Position source (VISION, ENCODERS, MANUAL, RESET)

**Field Coordinate System:**
- Field dimensions: 16.54m x 8.21m (2024 FRC field)
- Origin at bottom-left corner (0, 0)
- Center at (8.27, 4.105)
- Quadrant detection (BLUE_LEFT, BLUE_RIGHT, RED_LEFT, RED_RIGHT)

**Multiple Position Sources:**
- **Vision**: High accuracy from Limelight (confidence 0.9)
- **Encoders**: Fallback with drift compensation (confidence decreases)
- **Manual**: User-set positions (confidence 1.0)
- **Reset**: Field center reset (confidence 0.0)

**SmartDashboard Integration:**
- Real-time position display
- Quality indicators (EXCELLENT, GOOD, FAIR, POOR, UNKNOWN)
- Distance from field center
- Field quadrant information

## NetworkTables Structure

**Table: `fieldPosition`**
```
fieldPosition/robotX: Double (meters)
fieldPosition/robotY: Double (meters)  
fieldPosition/robotRotation: Double (degrees)
fieldPosition/timestamp: Double (seconds)
fieldPosition/confidence: Double (0.0 to 1.0)
fieldPosition/positionSource: String ("VISION", "ENCODERS", etc.)
```

**SmartDashboard Values:**
```
Field/RobotX: Current X position
Field/RobotY: Current Y position
Field/RobotRotation: Current rotation
Field/PositionSource: Data source
Field/Confidence: Position confidence
Field/DistanceFromCenter: Distance to center
Field/Quadrant: Field quadrant
Field/Quality: Position quality
```

## Controller Mapping

**Driver Controller:**
- **LEFT STICK Button**: Reset field position to center
- **START Button**: Limelight connection test
- **BACK Button**: Continuous Limelight diagnostic

**Operator Controller:**
- **B Button**: Auto-shoot (uses position for targeting)
- **A Button**: Toggle pulsing shooter
- **Y Button**: Manual continuous shooting

## Usage Instructions

### 1. Basic Position Tracking
The system automatically updates position in real-time:
- When Limelight has target → Uses VISION data
- When no target → Falls back to ENCODERS
- Position published to NetworkTables every 20ms

### 2. Reset Position
1. Press **LEFT STICK** button on driver controller
2. Robot position resets to field center (8.27, 4.105)
3. Confidence resets to 0.0
4. Position source set to "RESET"

### 3. Monitor Position
**SmartDashboard:**
- Watch `Field/RobotX` and `Field/RobotY` for coordinates
- Check `Field/Quality` for position accuracy
- Monitor `Field/Confidence` for data reliability

**NetworkTables:**
- Use tools like Shuffleboard or Glass
- Connect to robot's NetworkTables
- Add fieldPosition table to dashboard

### 4. Visualization Options

**Shuffleboard:**
1. Open Shuffleboard
2. Add "NetworkTables" widget
3. Select "fieldPosition" table
4. Create custom displays for X, Y, rotation

**Custom Dashboard:**
```python
# Example Python dashboard
import networktables as nt
nt.initialize(server='10.TE.AM.XX')

field_table = nt.get_table('fieldPosition')
x = field_table.getEntry('robotX').getDouble(0.0)
y = field_table.getEntry('robotY').getDouble(0.0)
rotation = field_table.getEntry('robotRotation').getDouble(0.0)
```

## Position Sources Logic

**Priority System:**
1. **VISION** (highest priority)
   - Limelight AprilTag detection
   - Confidence: 0.9
   - Updates when target available

2. **ENCODERS** (fallback)
   - Wheel encoder integration
   - Confidence decreases over time
   - Used when no vision data

3. **MANUAL** (user input)
   - Set via setPosition() method
   - Confidence: 1.0
   - Source: "MANUAL"

4. **RESET** (system reset)
   - Field center position
   - Confidence: 0.0
   - Source: "RESET"

## Field Coordinate System

**Dimensions:**
- Length: 16.54 meters (54 feet 1 inch)
- Width: 8.21 meters (26 feet 11 inches)
- Center: (8.27, 4.105)

**Quadrants:**
- **BLUE_LEFT**: X < 8.27, Y < 4.105
- **BLUE_RIGHT**: X ≥ 8.27, Y < 4.105  
- **RED_LEFT**: X < 8.27, Y ≥ 4.105
- **RED_RIGHT**: X ≥ 8.27, Y ≥ 4.105

**Coordinate Examples:**
- Bottom-left corner: (0, 0)
- Bottom-right corner: (16.54, 0)
- Top-left corner: (0, 8.21)
- Top-right corner: (16.54, 8.21)
- Field center: (8.27, 4.105)

## Quality Indicators

**Confidence Levels:**
- **EXCELLENT**: ≥ 0.8 (vision data available)
- **GOOD**: ≥ 0.6 (recent vision update)
- **FAIR**: ≥ 0.4 (encoder fallback)
- **POOR**: ≥ 0.2 (old data)
- **UNKNOWN**: < 0.2 (no reliable data)

**Position Quality:**
- High quality = vision + recent target
- Medium quality = recent encoder data
- Low quality = old or missing data

## Troubleshooting

**Position Not Updating:**
1. Check Limelight connection (press START button)
2. Verify NetworkTables connection
3. Check SmartDashboard values

**Poor Quality:**
1. Point Limelight at AprilTag
2. Check lighting conditions
3. Verify target distance

**Incorrect Position:**
1. Press LEFT STICK to reset to center
2. Move robot to known position
3. Verify coordinate system

**NetworkTables Issues:**
1. Check robot IP address
2. Verify NetworkTables server running
3. Check firewall settings

## Advanced Usage

**Custom Position Setting:**
```java
// Set robot to specific position
fieldPositionSystem.setPosition(5.0, 3.0, 90.0); // (x, y, rotation)
```

**Position Monitoring:**
```java
// Get current position
Pose2d currentPose = fieldPositionSystem.getRobotPose();
double confidence = fieldPositionSystem.getConfidence();
String source = fieldPositionSystem.getPositionSource();
```

**Integration with Autonomous:**
```java
// Use position for autonomous decisions
if (fieldPositionSystem.getDistanceFromCenter() < 2.0) {
    // Near center - execute center strategy
} else {
    // Far from center - execute field strategy
}
```

This system provides real-time robot position visualization that can be used for:
- Driver awareness during teleop
- Autonomous path planning
- Strategy decision making
- Debugging and development
- Match analysis and review
