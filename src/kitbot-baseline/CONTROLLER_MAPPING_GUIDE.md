# Controller Mapping Guide

## Overview
Complete button mapping for both driver and operator controllers with shooting position selection via D-pad.

## Driver Controller

### Diagnostic Controls
- **BACK Button**: Limelight continuous diagnostic
- **START Button**: Quick Limelight connection test  
- **LEFT STICK Button**: Reset field position to center

### Driving
- **Left Stick**: Forward/backward movement
- **Right Stick**: Rotation (turning)

## Operator Controller - Shooting Controls

### D-pad Position Selection
- **D-pad UP**: Cycle through Speaker positions
  - Speaker Center → Speaker Left → Speaker Right
- **D-pad RIGHT**: Next shooting position in list
  - Cycles through all 10 positions
- **D-pad DOWN**: Cycle through Stage positions  
  - Stage Left → Stage Center → Stage Right
- **D-pad LEFT**: Previous shooting position in list
  - Cycles backwards through all positions

### Shooting Actions
- **X Button**: Shoot at selected position
  - Uses position-based calculations
  - Automatic aiming and pulsing shooter
  - Distance and confidence validation
- **B Button**: Simple auto-shoot (legacy)
  - Basic Limelight targeting
  - Pulsing shooter
- **Y Button**: Manual continuous shooting
  - Holds shooter on continuously
  - No pulsing pattern
- **A Button**: Toggle pulsing shooter
  - Start/stop 3s ON/0.5s OFF pattern

### Fuel Management
- **Left Bumper**: Manual intake
  - Runs intake motor while held
- **Right Bumper**: Manual spin-up
  - Runs shooter motor while held

## Shooting Positions

### Available Positions (10 total)
1. **Speaker Center** (8.02, 0.22) - 2.5m preferred distance
2. **Speaker Left** (7.5, 0.5) - 2.8m preferred distance  
3. **Speaker Right** (8.5, 0.5) - 2.8m preferred distance
4. **Amp** (1.5, 7.0) - 1.5m preferred distance
5. **Stage Left** (4.5, 2.5) - 4.0m preferred distance
6. **Stage Center** (4.5, 4.1) - 4.0m preferred distance
7. **Stage Right** (4.5, 5.7) - 4.0m preferred distance
8. **Source Side** (15.0, 1.0) - 6.0m preferred distance
9. **Wing Position** (10.0, 2.0) - 3.5m preferred distance
10. **Podium** (2.0, 4.1) - 2.0m preferred distance

### D-pad Navigation Logic

**D-pad UP (Speaker Cycle):**
```
Speaker Center → Speaker Left → Speaker Right → Speaker Center
```

**D-pad DOWN (Stage Cycle):**
```
Stage Left → Stage Center → Stage Right → Stage Left
```

**D-pad RIGHT (Next Position):**
```
Speaker Center → Speaker Left → Speaker Right → Amp → Stage Left → ...
```

**D-pad LEFT (Previous Position):**
```
Current → Previous → Previous → ... (reverse order)
```

## SmartDashboard Display

### Shooting Position Info
```
Shooting/CurrentPosition: "Speaker Center"
Shooting/PositionX: 8.02 meters
Shooting/PositionY: 0.22 meters
Shooting/PositionRotation: 0 degrees
Shooting/PreferredDistance: 2.5 meters
```

### Real-time Shooting Data
```
Shooting/Distance: 2.3 meters (from Limelight)
Shooting/OptimalAngle: 35.0 degrees
Shooting/Confidence: 0.85 (0.0-1.0)
Shooting/Quality: "EXCELLENT"
```

## Usage Examples

### Example 1: Speaker Shooting
1. **D-pad UP** → Select "Speaker Center"
2. **X Button** → Shoot at speaker center
3. System automatically aims and shoots with pulsing pattern

### Example 2: Stage Position Shooting
1. **D-pad DOWN** → Select "Stage Center"  
2. **X Button** → Shoot at stage center
3. Robot moves to optimal 4.0m distance

### Example 3: Quick Position Change
1. **D-pad RIGHT** → Next position
2. **D-pad RIGHT** → Next position
3. **X Button** → Shoot at new position

### Example 4: Manual Override
1. **A Button** → Start pulsing shooter manually
2. **Y Button** → Continuous shooting if needed
3. **A Button** → Stop pulsing shooter

## Shooting Quality Indicators

### Confidence Levels
- **EXCELLENT** (≥0.8): Perfect distance, high accuracy
- **GOOD** (≥0.6): Good distance, reliable shooting
- **FAIR** (≥0.4): Acceptable distance, moderate accuracy
- **POOR** (≥0.2): Suboptimal distance, lower accuracy
- **BAD** (<0.2): Wrong distance, unreliable shooting

### Position Validation
- System checks if robot is within ±1.0m of preferred distance
- Provides feedback for position adjustments
- Automatic aiming when in good position

## Advanced Features

### Position-Based Calculations
- Each position has preferred shooting distance
- Optimal angle calculated based on actual distance
- Confidence based on distance match
- Automatic movement suggestions

### Integration with Field Position
- Uses field coordinate system for positioning
- Integrates with NetworkTables position data
- Supports autonomous path planning
- Real-time position tracking

### Error Handling
- Low confidence → Position adjustment suggestions
- No target → Waits for Limelight detection
- Target lost → Stops shooting automatically
- Connection issues → Falls back to manual shooting

## Troubleshooting

### D-pad Not Working
1. Check controller connection
2. Verify operator controller (not driver)
3. Check console for position selection messages

### Shooting Not Working
1. Press START button to test Limelight
2. Check if target is detected
3. Verify confidence level in SmartDashboard

### Wrong Position Selected
1. Use D-pad LEFT/RIGHT to navigate
2. Check SmartDashboard for current position
3. Press LEFT STICK to reset field position

### Poor Shooting Quality
1. Move robot to preferred distance
2. Check lighting conditions for Limelight
3. Verify AprilTag is visible

This system provides intuitive D-pad control for selecting shooting positions with automatic distance-based shooting calculations and real-time feedback.
