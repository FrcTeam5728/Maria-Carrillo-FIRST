package frc.robot.config;

import java.io.File;
import java.io.FileReader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

// Config file parsing is handled with a simple CSV-style fallback to avoid an extra JSON dependency.

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.utils.DashboardLogger;

public class ButtonManager {
    private final RobotContainer robotContainer;
    private final Path configPath;
    private final ControllerFactory.ControllerType controllerType;
    private final CommandXboxController controller;
    private final Map<String, Trigger> buttonMap = new HashMap<>();

    public ButtonManager(RobotContainer robotContainer, String controllerType, int port) {
        this.robotContainer = robotContainer;
        this.controllerType = ControllerFactory.detectControllerType(controllerType);
        this.controller = ControllerFactory.createController(this.controllerType, port);
        this.configPath = ControllerFactory.getConfigPath(controllerType);

        // Initialize button map based on controller type
        initializeButtonMap();
    }
    
    private void initializeButtonMap() {
        // Common buttons across all controllers
        buttonMap.put("A", controller.a());
        buttonMap.put("B", controller.b());
        buttonMap.put("X", controller.x());
        buttonMap.put("Y", controller.y());
        buttonMap.put("START", controller.start());
        buttonMap.put("BACK", controller.back());
        
        // Controller-specific buttons (Xbox mapping). For unknown controller types we keep
        // the common A/B/X/Y/START/BACK buttons and avoid PS4-specific APIs to stay
        // independent of optional vendor classes.
        if (controllerType == ControllerFactory.ControllerType.XBOX) {
            buttonMap.put("LEFT_BUMPER", ((CommandXboxController) controller).leftBumper());
            buttonMap.put("RIGHT_BUMPER", ((CommandXboxController) controller).rightBumper());
            buttonMap.put("LEFT_STICK", ((CommandXboxController) controller).leftStick());
            buttonMap.put("RIGHT_STICK", ((CommandXboxController) controller).rightStick());
        }
        // Add more controller types as needed
    }

    public void configureButtons() {
        // Dynamic button configs are not parsed in this build to avoid adding
        // an external JSON dependency. Use the deploy-time defaults or bind
        // buttons programmatically in RobotContainer instead.
        DashboardLogger.putString("ButtonManager/Info", "Button config parsing is disabled in this build. No dynamic bindings loaded.");
    }

    // Dynamic binding is disabled in this build. If you need bindings loaded
    // from a config file, implement parsing here or bind programmatically in
    // RobotContainer.

    // Get the controller instance for direct access if needed
    public CommandXboxController getController() {
        return controller;
    }
    
    // Get the controller type
    public ControllerFactory.ControllerType getControllerType() {
        return controllerType;
    }

    private Command getCommandForConfig(String commandName) {
        // Delegate command creation to the robot container
        // This allows the RobotContainer to define what each command does
        return robotContainer.getCommandForButton(commandName);
    }
}
