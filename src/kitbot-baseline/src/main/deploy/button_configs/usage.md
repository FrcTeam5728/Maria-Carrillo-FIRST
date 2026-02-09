# Button Configuration System

This system allows you to configure button bindings through JSON configuration files, making it easy to modify button mappings without changing code.

## Configuration Files

- Location: `deploy/button_configs/`
- Example: `xbox_config.json`

## Configuration Format

```json
{
  "controllerType": "XBOX",
  "buttonMappings": [
    {
      "button": "A",
      "commandType": "WHILE_HELD",
      "command": "eject"
    },
    {
      "button": "LEFT_BUMPER",
      "commandType": "WHILE_HELD",
      "command": "intake"
    },
    {
      "button": "RIGHT_BUMPER",
      "commandType": "WHILE_HELD",
      "command": "launch"
    }
  ]
}
```

## Available Buttons
- `A`, `B`, `X`, `Y` - Face buttons
- `LEFT_BUMPER`, `RIGHT_BUMPER` - Bumper buttons
- `BACK`, `START` - Menu buttons
- `LEFT_STICK`, `RIGHT_STICK` - Stick press buttons

## Command Types
- `WHEN_PRESSED` - Triggers once when button is pressed
- `WHEN_RELEASED` - Triggers once when button is released
- `WHILE_HELD` - Runs while the button is held down
- `TOGGLE_WHEN_PRESSED` - Toggles on/off with each button press

## Adding New Mappings
1. Edit the appropriate config file in `deploy/button_configs/`
2. Add a new entry to the `buttonMappings` array
3. Set the `button`, `commandType`, and `command` values
4. Deploy the code to the robot

## Notes
- Changes to the config file take effect on robot code deployment
- The system automatically loads the configuration when the robot starts
- Invalid configurations will be logged to the console