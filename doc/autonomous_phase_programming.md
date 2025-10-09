# autonomous phase programming

The autonomous phase is used in the first 30 seconds at the start of the game, however, it can be used at any point after as well. It is useful for easily automatable behaviors including

- Drive to a position
- April tag alignment
- Intake procedure
- And probably more

## How to use it

In order to use the autonomous phase, you need to edit the the json file "src/swerve-baseline/src/main/java/frc/robot/commands/AutonomousDirections.json". The file should have the following format: 

```json
[
    {
      "command": "DriveToPosition",
      "parameters": {
        "x_meters": 1.524,
        "y_meters": 0.0,
        "rotation_degrees": 0.0
      }
    },
    {
      "command": "Wait",
      "parameters": {
        "seconds": 1.0
      }
    }
]
```

Currently, only wait and drive to position are supported. I will probably add more commands in the future.
