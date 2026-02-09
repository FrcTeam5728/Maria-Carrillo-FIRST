package frc.robot.config;

import java.io.File;
import java.io.FileReader;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.FuelSubsystem;

public class ButtonManager {
    private final RobotContainer robotContainer;
    private final Path configPath;
    private final ControllerFactory.ControllerType controllerType;
    private final CommandGenericHID controller;
    private final Map<String, Trigger> buttonMap = new HashMap<>();

    public ButtonManager(RobotContainer robotContainer, String controllerType, int port) {
        this.robotController = robotContainer;
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
        
        // Controller-specific buttons
        if (controllerType == ControllerFactory.ControllerType.XBOX) {
            buttonMap.put("LEFT_BUMPER", ((CommandXboxController) controller).leftBumper());
            buttonMap.put("RIGHT_BUMPER", ((CommandXboxController) controller).rightBumper());
            buttonMap.put("LEFT_STICK", ((CommandXboxController) controller).leftStick());
            buttonMap.put("RIGHT_STICK", ((CommandXboxController) controller).rightStick());
        } else if (controllerType == ControllerFactory.ControllerType.PS4) {
            buttonMap.put("LEFT_BUMPER", ((CommandPS4Controller) controller).L1());
            buttonMap.put("RIGHT_BUMPER", ((CommandPS4Controller) controller).R1());
            buttonMap.put("LEFT_STICK", ((CommandPS4Controller) controller).L3());
            buttonMap.put("RIGHT_STICK", ((CommandPS4Controller) controller).R3());
        }
        // Add more controller types as needed
    }

    public void configureButtons() {
        try (FileReader reader = new FileReader(configPath.toFile())) {
            JSONObject config = new JSONObject(new JSONTokener(reader));
            JSONArray buttonMappings = config.getJSONArray("buttonMappings");
            
            for (int i = 0; i < buttonMappings.length(); i++) {
                try {
                    ButtonConfig buttonConfig = ButtonConfig.fromJson(buttonMappings.getJSONObject(i));
                    bindButton(buttonConfig);
                } catch (Exception e) {
                    System.err.println("Error binding button config at index " + i + ": " + e.getMessage());
                }
            }
        } catch (Exception e) {
            System.err.println("Failed to load button config from " + configPath + ": " + e.getMessage());
            e.printStackTrace();
        }
    }

    private void bindButton(ButtonConfig config) {
        Command command = getCommandForConfig(config.command);
        Trigger trigger = buttonMap.get(config.button);

        if (trigger == null) {
            System.err.println("Button not found: " + config.button);
            return;
        }
        if (command == null) {
            System.err.println("Command not found: " + config.command);
            return;
        }

        switch (config.commandType) {
            case WHEN_PRESSED:
                trigger.onTrue(command);
                break;
            case WHEN_RELEASED:
                trigger.onFalse(command);
                break;
            case WHILE_HELD:
                trigger.whileTrue(command);
                break;
            case TOGGLE_WHEN_PRESSED:
                trigger.toggleOnTrue(command);
                break;
        }
        
        System.out.println("Bound " + config.button + " to " + config.command + " (" + config.commandType + ")");
    }

    // Get the controller instance for direct access if needed
    public CommandGenericHID getController() {
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
