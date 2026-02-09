package frc.robot.commands;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.ButtonConfig;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;

/**
 * Manages all commands and their bindings.
 * Acts as a central place to register and retrieve commands.
 */
public class CommandManager {
    private static CommandManager instance;
    private final Map<String, Command> commandRegistry;
    private final Map<Integer, CommandJoystick> joysticks;
    private final CommandXboxController driverController;
    private final SwerveSubsystem drivebase;

    // Command instances
    private Command driveFieldOrientedDirectAngle;
    private Command driveFieldOrientedAngularVelocity;
    private Command driveRobotOrientedAngularVelocity;
    private Command driveSetpointGen;
    private Command driveFieldOrientedDirectAngleKeyboard;
    private Command driveFieldOrientedAngularVelocityKeyboard;
    private Command driveSetpointGenKeyboard;
    
    // Movement commands
    private Command moveForward;
    private Command moveBackward;
    private Command strafeLeft;
    private Command strafeRight;
    private Command rotateLeft;
    private Command rotateRight;
    private Command stopMovement;

    private CommandManager(SwerveSubsystem drivebase, CommandXboxController driverController) {
        this.drivebase = drivebase;
        this.driverController = driverController;
        this.commandRegistry = new HashMap<>();
        this.joysticks = new HashMap<>();
        this.joysticks.put(0, driverController); // Add default controller
        
        // Initialize all commands
        initializeCommands();
    }

    /**
     * Get the singleton instance of CommandManager
     */
    public static synchronized CommandManager getInstance(SwerveSubsystem drivebase, CommandXboxController driverController) {
        if (instance == null) {
            instance = new CommandManager(drivebase, driverController);
        }
        return instance;
    }

    /**
     * Initialize all commands
     */
    private void initializeCommands() {
        // Initialize drive commands
        driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(drivebase.getDriveDirectAngleStream());
        driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(drivebase.getDriveAngularVelocityStream());
        driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(drivebase.getDriveRobotOrientedStream());
        driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(drivebase.getDriveDirectAngleStream());
        
        // Initialize keyboard-specific commands
        driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(drivebase.getDriveDirectAngleKeyboardStream());
        driveFieldOrientedAngularVelocityKeyboard = drivebase.driveFieldOriented(drivebase.getDriveAngularVelocityKeyboardStream());
        driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(drivebase.getDriveDirectAngleKeyboardStream());

        // Initialize movement commands
        initializeMovementCommands();
        
        // Register drive mode commands
        registerCommand("drive_field_oriented_direct_angle", driveFieldOrientedDirectAngle);
        registerCommand("drive_field_oriented_angular_velocity", driveFieldOrientedAngularVelocity);
        registerCommand("drive_robot_oriented_angular_velocity", driveRobotOrientedAngularVelocity);
        registerCommand("drive_setpoint_gen", driveSetpointGen);
        registerCommand("drive_field_oriented_direct_angle_keyboard", driveFieldOrientedDirectAngleKeyboard);
        registerCommand("drive_field_oriented_angular_velocity_keyboard", driveFieldOrientedAngularVelocityKeyboard);
        registerCommand("drive_setpoint_gen_keyboard", driveSetpointGenKeyboard);
        
        // Register movement commands
        registerCommand("move_forward", moveForward);
        registerCommand("move_backward", moveBackward);
        registerCommand("strafe_left", strafeLeft);
        registerCommand("strafe_right", strafeRight);
        registerCommand("rotate_left", rotateLeft);
        registerCommand("rotate_right", rotateRight);
        registerCommand("stop_movement", stopMovement);
    }

    /**
     * Register a command with a unique name
     */
    public void registerCommand(String name, Command command) {
        commandRegistry.put(name, command);
    }

    /**
     * Get a registered command by name
     */
    public Command getCommand(String name) {
        return commandRegistry.get(name);
    }

    /**
     * Get the default drive command based on the current mode
     */
    public Command getDefaultDriveCommand() {
        if (RobotBase.isSimulation()) {
            return getCommand("drive_field_oriented_direct_angle_keyboard");
        } else {
            return getCommand("drive_field_oriented_angular_velocity");
        }
    }

    /**
     * Get the drive to pose command
     */
    public Command getDriveToPoseCommand() {
        return drivebase.driveToPose(
            new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))
        );
    }

    /**
     * Load and apply button configurations from a JSON file
     */
    public void loadButtonConfigs(String configPath) {
        try {
            File configFile = new File(Filesystem.getDeployDirectory(), configPath);
            ObjectMapper mapper = new ObjectMapper();
            List<ButtonConfig> configs = mapper.readValue(
                configFile, 
                new TypeReference<List<ButtonConfig>>() {}
            );
            
            applyButtonConfigs(configs);
        } catch (IOException e) {
            System.err.println("Failed to load button configs: " + e.getMessage());
            e.printStackTrace();
        }
    }
    
    /**
     * Apply button configurations to the controllers
     */
    private void applyButtonConfigs(List<ButtonConfig> configs) {
        for (ButtonConfig config : configs) {
            CommandJoystick joystick = joysticks.computeIfAbsent(
                config.getPort(), 
                port -> new CommandJoystick(port)
            );
            
            Trigger button = joystick.button(config.getButton());
            Command command = commandRegistry.get(config.getCommand());
            
            if (command == null) {
                System.err.println("Command not found: " + config.getCommand());
                continue;
            }
            
            if (config.isWhenPressed()) {
                button.onTrue(command);
            }
            if (config.isWhileHeld()) {
                button.whileTrue(command);
            }
            if (config.isWhenReleased()) {
                button.onFalse(command);
            }
        }
    }
    
    /**
     * Register a joystick to be used for button bindings
     */
    public void registerJoystick(int port, CommandJoystick joystick) {
        joysticks.put(port, joystick);
    }
    
    /**
     * Initialize movement commands with default speeds
     */
    private void initializeMovementCommands() {
        double moveSpeed = 0.5;  // 50% speed for movement
        double rotateSpeed = 0.3; // 30% speed for rotation
        
        // Create movement commands
        moveForward = Commands.startEnd(
            () -> drivebase.drive(moveSpeed, 0, 0, false, false),
            drivebase::stop,
            drivebase
        ).withName("Move Forward");
        
        moveBackward = Commands.startEnd(
            () -> drivebase.drive(-moveSpeed, 0, 0, false, false),
            drivebase::stop,
            drivebase
        ).withName("Move Backward");
        
        strafeLeft = Commands.startEnd(
            () -> drivebase.drive(0, -moveSpeed, 0, false, false),
            drivebase::stop,
            drivebase
        ).withName("Strafe Left");
        
        strafeRight = Commands.startEnd(
            () -> drivebase.drive(0, moveSpeed, 0, false, false),
            drivebase::stop,
            drivebase
        ).withName("Strafe Right");
        
        rotateLeft = Commands.startEnd(
            () -> drivebase.drive(0, 0, -rotateSpeed, false, false),
            drivebase::stop,
            drivebase
        ).withName("Rotate Left");
        
        rotateRight = Commands.startEnd(
            () -> drivebase.drive(0, 0, rotateSpeed, false, false),
            drivebase::stop,
            drivebase
        ).withName("Rotate Right");
        
        stopMovement = Commands.runOnce(drivebase::stop, drivebase)
            .withName("Stop Movement");
    }
    
    /**
     * Get the default movement command based on the current mode
     */
    public Command getDefaultMovementCommand() {
        if (RobotBase.isSimulation()) {
            return getCommand("drive_field_oriented_direct_angle_keyboard");
        } else {
            return getCommand("drive_field_oriented_angular_velocity");
        }
    }
    
    public void configureButtonBindings() {
        // Default implementation does nothing
        // Button bindings are now loaded from config files
    }

    private void configureTestModeBindings() {
        driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        driverController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
        driverController.start().onTrue(Commands.runOnce(drivebase::zeroGyro));
        driverController.back().whileTrue(drivebase.centerModulesCommand());
    }

    private void configureTeleopBindings() {
        driverController.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
        driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
        driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    }
}
