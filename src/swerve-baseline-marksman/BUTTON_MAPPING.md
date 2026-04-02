# Button and Joystick Mappings

This document outlines the default button and joystick mappings for the robot controls.

## Default Controller (Xbox Controller) - Port 0

### Drive Mode Selection
| Button | Command | Action |
|--------|---------|--------|
| A (1)  | `drive_field_oriented_direct_angle` | Toggle field-oriented direct angle drive |
| B (2)  | `drive_field_oriented_angular_velocity` | Toggle field-oriented angular velocity drive |
| X (3)  | `drive_robot_oriented_angular_velocity` | Toggle robot-oriented angular velocity drive |

### Movement Controls
| Control | Button | Command | Action |
|---------|--------|---------|--------|
| D-Pad Up | 5 | `move_forward` | Move robot forward |
| D-Pad Down | 6 | `move_backward` | Move robot backward |
| D-Pad Left | 7 | `strafe_left` | Strafe left |
| D-Pad Right | 8 | `strafe_right` | Strafe right |
| Left Bumper | 9 | `rotate_left` | Rotate counter-clockwise |
| Right Bumper | 10 | `rotate_right` | Rotate clockwise |
| Back | 5 (press) | `stop_movement` | Stop all movement |

### Utility Controls
| Button | Command | Action |
|--------|---------|--------|
| Start  | `zero_gyro` | Reset the gyro (robot heading) |

### Joysticks/Axes (when not using movement controls)
| Axis | Function |
|------|----------|
| Left Y | Forward/Backward movement (inverted) |
| Left X | Strafe Left/Right (inverted) |
| Right X | Rotation (Z-axis) |
| Right Y | (Not used by default) |

## Additional Joysticks

Additional joysticks can be added by registering them with the `CommandManager` using:

```java
CommandJoystick joystick = new CommandJoystick(portNumber);
CommandManager.getInstance().registerJoystick(portNumber, joystick);
```

## Button Configuration File

Button mappings can be customized by editing the `src/main/deploy/button_config.json` file. The format is:

```json
[
  {
    "port": 0,
    "button": 1,
    "command": "command_name",
    "whenPressed": true,
    "whileHeld": false,
    "whenReleased": false
  }
]
```

### Configuration Options:
- `port`: The joystick/controller port number
- `button`: The button number (1 = A, 2 = B, etc. for Xbox controllers)
- `command`: The name of the command to execute (must be registered in CommandManager)
- `whenPressed`: Trigger command when button is pressed
- `whileHeld`: Trigger command while button is held
- `whenReleased`: Trigger command when button is released

## Available Commands

### Drive Modes
- `drive_field_oriented_direct_angle` - Field-oriented drive with direct angle control
- `drive_field_oriented_angular_velocity` - Field-oriented drive with angular velocity control
- `drive_robot_oriented_angular_velocity` - Robot-oriented drive with angular velocity control
- `drive_setpoint_gen` - Drive with setpoint generator

### Movement Commands
- `move_forward` - Move robot forward at 50% speed
- `move_backward` - Move robot backward at 50% speed
- `strafe_left` - Strafe left at 50% speed
- `strafe_right` - Strafe right at 50% speed
- `rotate_left` - Rotate counter-clockwise at 30% speed
- `rotate_right` - Rotate clockwise at 30% speed
- `stop_movement` - Immediately stop all robot movement

### Utility Commands
- `zero_gyro` - Reset the gyro (robot heading)
- `center_modules` - Center all swerve modules

## Adding New Commands

To add a new command that can be bound to buttons:

1. Create your command class
2. Register it in the `CommandManager`:
   ```java
   commandManager.registerCommand("my_command", new MyCommand());
   ```
3. Add it to your button configuration file

## Troubleshooting

- If a button doesn't work, check the Driver Station for error messages
- Verify the command name in the config file matches the registered command name
- Ensure the correct port number is used for each controller
- Check that the button number corresponds to the correct button on your controller
