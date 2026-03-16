# Centralized Limelight and Pulsing Shooting System

## Overview
Replaced multiple programs calling Limelight functions with a single centralized system. Added pulsing shooter with 3-second ON, 0.5-second OFF firing pattern.

## System Architecture

### ✅ Single LimelightSubsystem
**Centralized Vision Data:**
- One authoritative source for all Limelight data
- Automatic data polling and caching in periodic()
- NetworkTables filtering and validation
- SmartDashboard integration
- Connection monitoring and error handling

**Key Features:**
- Eliminates multiple programs calling Limelight functions
- Cached data reduces NetworkTables load
- Built-in target filtering (area, offset validation)
- Automatic connection status tracking

### ✅ PulsingShooterSubsystem
**Controlled Firing Pattern:**
- 3 seconds ON (firing fuel)
- 0.5 seconds OFF (pause)
- Repeats automatically when enabled
- Provides consistent fuel delivery

**Key Features:**
- Automatic pulse timing (3s ON, 0.5s OFF)
- Pulse counting and statistics
- Manual override capability
- SmartDashboard monitoring
- Safety interlocks and speed limits

### ✅ SimpleAutoShootCommand
**Simplified Shooting Logic:**
- Uses centralized Limelight data
- Automatic target acquisition and aiming
- Integrates with pulsing shooter
- Clear status feedback

## Controller Mapping

### Operator Controller:
- **B Button**: Auto-shoot (aims and shoots automatically)
- **Y Button**: Manual continuous shooting
- **A Button**: Toggle pulsing shooter
- **Left Bumper**: Manual intake
- **Right Bumper**: Manual spin-up

## Limelight NetworkTables Integration

### Single Data Source:
```
LimelightSubsystem (centralized)
├── Updates all NetworkTables data in periodic()
├── Provides cached data to all other systems
├── Handles connection monitoring
└── Manages data validation and filtering
```

### Available Data:
- `hasTarget()` - Target detection status
- `getHorizontalOffset()` - Horizontal angle to target
- `getVerticalOffset()` - Vertical angle to target
- `getTargetId()` - AprilTag ID
- `getDistance()` - Approximate distance
- `isConnected()` - Connection status
- `getTargetInfo()` - Complete target information

### SmartDashboard Values:
```
Limelight/Connected: Boolean
Limelight/HasTarget: Boolean
Limelight/HorizontalOffset: Degrees
Limelight/VerticalOffset: Degrees
Limelight/TargetArea: Percentage
Limelight/TargetID: AprilTag number
Limelight/Distance: Meters
Limelight/Latency: Milliseconds
```

## Pulsing Shooter Pattern

### Timing Diagram:
```
|---- ON (3s) ----|-- OFF (0.5s) --|---- ON (3s) ----|-- OFF (0.5s) --|...
   Firing fuel      No fuel fired      Firing fuel      No fuel fired
```

### Motor Speeds:
- **ON Phase**: Shooter 100%, Feeder 80%
- **OFF Phase**: Shooter 30% (maintaining spin), Feeder 0%

### Statistics:
- Pulse count tracking
- Total shots fired
- Shots per second calculation
- Cycle position monitoring

### SmartDashboard Values:
```
Shooter/IsShooting: Boolean
Shooter/IsPulsing: Boolean
Shooter/IsPulseOn: Boolean
Shooter/ShooterSpeed: Percentage
Shooter/FeederSpeed: Percentage
Shooter/PulseCount: Number
Shooter/TotalShots: Number
Shooter/CyclePosition: 0.0-1.0
Shooter/ShotsPerSecond: Rate
```

## Benefits of Centralized Approach

### ✅ Single Source of Truth:
- One LimelightSubsystem handles all vision data
- Eliminates duplicate NetworkTables access
- Consistent data across all systems
- Reduced network traffic

### ✅ Improved Performance:
- Cached data reduces NetworkTables calls
- Automatic filtering prevents invalid data
- Connection monitoring prevents errors
- Efficient data access patterns

### ✅ Better Debugging:
- Centralized logging and error handling
- Single point for vision system configuration
- Comprehensive SmartDashboard integration
- Clear status reporting

### ✅ Pulsing Advantages:
- Controlled fuel delivery prevents jams
- Consistent shooting pattern
- Reduced motor wear with OFF periods
- Predictable timing for strategy

## Usage Instructions

### Automatic Shooting:
1. **Press B** to start auto-shoot
2. System automatically finds and aims at target
3. Pulsing shooter fires with 3s ON/0.5s OFF pattern
4. Continues until manually cancelled

### Manual Control:
1. **Press A** to toggle pulsing shooter
2. **Hold Y** for continuous shooting
3. **Use bumpers** for manual intake/spin-up

### Monitoring:
- Watch SmartDashboard for real-time status
- Check console for detailed logging
- Monitor pulse count and timing

## Implementation Details

### Data Flow:
```
Limelight Hardware → NetworkTables → LimelightSubsystem → Other Commands
```

### Periodic Updates:
- LimelightSubsystem: Updates cached data from NetworkTables
- PulsingShooterSubsystem: Manages 3s/0.5s pulse pattern
- SimpleAutoShootCommand: Uses cached data for shooting logic

### Error Handling:
- Connection timeout detection
- Invalid data filtering
- Automatic fallback to safe states
- Comprehensive error logging

## Expected Behavior

### Normal Operation:
1. Limelight detects target → LimelightSubsystem caches data
2. User presses B → SimpleAutoShootCommand starts
3. Robot aims automatically → Aligns with target
4. Pulsing shooter starts → 3s ON/0.5s OFF pattern
5. Fuel delivered consistently → Target hit

### Error Conditions:
- No target: "No target - waiting"
- Lost connection: "Limelight not responding"
- Invalid data: Filtered out automatically
- Manual override: Available at any time

This centralized approach provides a much cleaner, more reliable system with better performance and easier debugging compared to multiple programs calling Limelight functions directly.
