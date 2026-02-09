package frc.robot.config;

import java.io.FileReader;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

// Filesystem import intentionally omitted; not used here
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
// FuelSubsystem not required by this manager; access commands via RobotContainer

public class ButtonManager {
    private final RobotContainer robotContainer;
    private final Path configPath;
    private final ControllerFactory.ControllerType controllerType;
    private final CommandGenericHID controller;
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
        // Initialize buttons per controller type to avoid calling methods
        // that don't exist on CommandGenericHID.
        if (controllerType == ControllerFactory.ControllerType.XBOX) {
            CommandXboxController xbox = (CommandXboxController) controller;
            // Standard face buttons
            buttonMap.put("A", xbox.a());
            buttonMap.put("B", xbox.b());
            buttonMap.put("X", xbox.x());
            buttonMap.put("Y", xbox.y());
            buttonMap.put("START", xbox.start());
            buttonMap.put("BACK", xbox.back());

            // Bumpers / sticks
            buttonMap.put("LEFT_BUMPER", xbox.leftBumper());
            buttonMap.put("RIGHT_BUMPER", xbox.rightBumper());
            buttonMap.put("LEFT_STICK", xbox.leftStick());
            buttonMap.put("RIGHT_STICK", xbox.rightStick());
        } else if (controllerType == ControllerFactory.ControllerType.PS4) {
            CommandPS4Controller ps4 = (CommandPS4Controller) controller;
            // PS4 naming differs; map common labels to PS4 equivalents where sensible
            try {
                buttonMap.put("A", ps4.cross());
                buttonMap.put("B", ps4.circle());
                buttonMap.put("X", ps4.square());
                buttonMap.put("Y", ps4.triangle());
            } catch (NoSuchMethodError | RuntimeException ignore) {
                // Some WPILib versions may name methods differently; ignore if unavailable
            }

            // Bumpers / sticks
            buttonMap.put("LEFT_BUMPER", ps4.L1());
            buttonMap.put("RIGHT_BUMPER", ps4.R1());
            buttonMap.put("LEFT_STICK", ps4.L3());
            buttonMap.put("RIGHT_STICK", ps4.R3());
        } else {
            // For joysticks or unknown types we don't populate face buttons by name.
            // Users can still access the raw controller via getController().
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
