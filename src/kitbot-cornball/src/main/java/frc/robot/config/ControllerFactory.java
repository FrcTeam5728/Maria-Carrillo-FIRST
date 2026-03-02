package frc.robot.config;

import java.nio.file.Path;
import java.nio.file.Paths;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerFactory {
    public enum ControllerType {
        XBOX,
        PS4,
        JOYSTICK
    }

    public static CommandGenericHID createController(ControllerType type, int port) {
        return switch (type) {
            case XBOX -> new CommandXboxController(port);
            case PS4 -> new CommandPS4Controller(port);
            case JOYSTICK -> new CommandJoystick(port);
        };
    }

    public static ControllerType detectControllerType(String configName) {
        String lowerName = configName.toLowerCase();
        if (lowerName.contains("xbox")) return ControllerType.XBOX;
        if (lowerName.contains("ps4") || lowerName.contains("playstation")) return ControllerType.PS4;
        return ControllerType.JOYSTICK; // Default to joystick if unknown
    }

    public static Path getConfigPath(String controllerType) {
        return Paths.get(
            Filesystem.getDeployDirectory().getAbsolutePath(),
            "button_configs",
            controllerType.toLowerCase() + "_config.json"
        );
    }
}
