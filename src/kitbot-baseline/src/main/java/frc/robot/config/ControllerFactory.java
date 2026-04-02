package frc.robot.config;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * Minimal controller factory to avoid optional dependencies on PS4/etc. classes.
 * This intentionally keeps a small surface so the code compiles without
 * adding extra Gradle dependencies.
 */
public final class ControllerFactory {
    public enum ControllerType {
        XBOX,
        GENERIC
    }

    public static ControllerType detectControllerType(String name) {
        if (name == null) {
            return ControllerType.GENERIC;
        }
        String n = name.trim().toLowerCase();
        if (n.contains("xbox")) {
            return ControllerType.XBOX;
        }
        return ControllerType.GENERIC;
    }

    public static CommandXboxController createController(ControllerType t, int port) {
        // We return a CommandXboxController for both XBOX and GENERIC to keep
        // the controller API available without pulling in other optional
        // controller classes.
        return new CommandXboxController(port);
    }

    public static Path getConfigPath(String controllerType) {
        // Default deploy path for button configs. The file may not exist; callers
        // should check. Keep this stable even if parsing is disabled.
        return Filesystem.getDeployDirectory().toPath().resolve("button_configs").resolve("xbox_config.json");
    }
}
