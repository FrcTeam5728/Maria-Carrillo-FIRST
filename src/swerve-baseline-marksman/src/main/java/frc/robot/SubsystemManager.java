package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CommandManager;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import java.util.HashMap;
import java.util.Map;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.SwerveStream;

/**
 * Manages all subsystems and their default commands.
 * Acts as a middleman between RobotContainer and individual subsystems.
 */
public class SubsystemManager {
    private static SubsystemManager instance;
    private final Map<Class<? extends SubsystemBase>, SubsystemBase> subsystems;
    private final CommandXboxController driverController;

    // Subsystems
    private SwerveSubsystem drivebase;

    // Command Manager
    private final CommandManager commandManager;
    private final AutoRegistry autoRegistry;
    
    // Input streams (package-private for CommandManager access)
    SwerveInputStream driveAngularVelocity;
    SwerveInputStream driveDirectAngle;
    SwerveInputStream driveRobotOriented;
    SwerveInputStream driveAngularVelocityKeyboard;
    SwerveInputStream driveDirectAngleKeyboard;

    private SubsystemManager() {
        subsystems = new HashMap<>();
        driverController = new CommandXboxController(0);
        
        // Initialize command manager first
        commandManager = CommandManager.getInstance(drivebase, driverController);
        
        // Initialize auto registry
        autoRegistry = new AutoRegistry(this, commandManager);
        
        // Initialize all subsystems (this will now use auto-registration)
        initializeSubsystems();
        
        // Configure default commands
        configureDefaultCommands();
    }

    /**
     * Get the singleton instance of SubsystemManager
     */
    public static synchronized SubsystemManager getInstance() {
        if (instance == null) {
            instance = new SubsystemManager();
        }
        return instance;
    }

    /**
     * Initialize all robot subsystems
     */
    private void initializeSubsystems() {
        try {
            // Initialize drivebase (manually since it needs special parameters)
            drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
            registerSubsystem(SwerveSubsystem.class, drivebase);
            
            // Initialize input streams
            initializeInputs();
            
            // Auto-register all other subsystems and commands
            autoRegistry.registerAll();
            
        } catch (Exception e) {
            System.err.println("Error initializing subsystems: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * Initialize all input streams
     */
    private void initializeInputs() {
        // Initialize input streams for drive control
        driveAngularVelocity = createAngularVelocityStream(false);
        driveDirectAngle = createDirectAngleStream(false);
        driveRobotOriented = createRobotOrientedStream();
        driveAngularVelocityKeyboard = createAngularVelocityStream(true);
        driveDirectAngleKeyboard = createDirectAngleStream(true);
    }

    private SwerveInputStream createAngularVelocityStream(boolean isKeyboard) {
        SwerveInputStream stream = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> (isKeyboard ? -1 : 1) * driverController.getLeftY() * -1,
            () -> (isKeyboard ? -1 : 1) * driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> isKeyboard ? driverController.getRawAxis(2) : driverController.getRightX())
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(!isKeyboard);
            
        if (isKeyboard) {
            stream.robotRelative(true);
        }
        
        return stream;
    }

    private SwerveInputStream createDirectAngleStream(boolean isKeyboard) {
        SwerveInputStream baseStream = isKeyboard ? driveAngularVelocityKeyboard : driveAngularVelocity;
        return baseStream.copy()
            .withControllerHeadingAxis(
                isKeyboard ? 
                    () -> Math.sin(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2) :
                    driverController::getRightX,
                isKeyboard ?
                    () -> Math.cos(driverController.getRawAxis(2) * Math.PI) * (Math.PI * 2) :
                    driverController::getRightY)
            .headingWhile(true)
            .translationHeadingOffset(isKeyboard)
            .translationHeadingOffset(Rotation2d.fromDegrees(0));
    }

    private SwerveInputStream createRobotOrientedStream() {
        return driveAngularVelocity.copy()
            .robotRelative(true)
            .allianceRelativeControl(false);
    }
    
    // Getters for input streams (used by CommandManager)
    public SwerveInputStream getDriveAngularVelocityStream() { return driveAngularVelocity; }
    public SwerveInputStream getDriveDirectAngleStream() { return driveDirectAngle; }
    public SwerveInputStream getDriveRobotOrientedStream() { return driveRobotOriented; }
    public SwerveInputStream getDriveAngularVelocityKeyboardStream() { return driveAngularVelocityKeyboard; }
    public SwerveInputStream getDriveDirectAngleKeyboardStream() { return driveDirectAngleKeyboard; }

    /**
     * Configure default commands for all subsystems
     */
    private void configureDefaultCommands() {
        drivebase.setDefaultCommand(commandManager.getDefaultDriveCommand());
    }

    /**
     * Configure button bindings for all subsystems
     */
    public void configureButtonBindings() {
        commandManager.configureButtonBindings();
    }

    /**
     * Register a subsystem with the manager
     */
    private <T extends SubsystemBase> void registerSubsystem(Class<T> type, T subsystem) {
        subsystems.put(type, subsystem);
    }

    /**
     * Get a registered subsystem by its type
     */
    @SuppressWarnings("unchecked")
    public <T extends SubsystemBase> T getSubsystem(Class<T> type) {
        return (T) subsystems.get(type);
    }

    /**
     * Get the drivebase subsystem
     */
    public SwerveSubsystem getDrivebase() {
        return drivebase;
    }

    /**
     * Get the driver controller
     */
    public CommandXboxController getDriverController() {
        return driverController;
    }

    /**
     * Get the autonomous command
     */
    public Command getAutonomousCommand() {
        return commandManager.getDriveToPoseCommand();
    }

    /**
     * Set motor brake mode
     */
    public void setMotorBrake(boolean brake) {
        drivebase.setMotorBrake(brake);
    }
}
