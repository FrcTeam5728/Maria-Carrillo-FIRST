// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.RampZigzagShootAuto;
import frc.robot.commands.SimpleAutoShootCommand;
import frc.robot.config.SimpleShuffleboardControls;
import frc.robot.config.PathPlannerAutoBuilder;
import frc.robot.subsystems.SimpleCameraSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;
import frc.robot.utils.CameraFeedBroadcaster;
import frc.robot.utils.DynamicUSBCameraServer;
import frc.robot.utils.LimelightCameraServer;
import frc.robot.utils.USBCameraServer;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem driveSubsystem = RobotSubsystemFactory.createDriveSubsystem();
  private final FuelSubsystem ballSubsystem = RobotSubsystemFactory.createFuelSubsystem();
  private final LimelightSubsystem limelightSubsystem = new LimelightSubsystem();
  private final PulsingShooterSubsystem shooterSubsystem = new PulsingShooterSubsystem();
  // Camera subsystems
  private final SimpleCameraSubsystem cameraServerSubsystem = new SimpleCameraSubsystem(); // Simplified constructor
  private final CameraFeedBroadcaster cameraFeedBroadcaster = new CameraFeedBroadcaster();

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize subsystems
    System.out.println("=== SIMPLIFIED ROBOT SYSTEM ===");
    System.out.println("Camera server, shooting, and driving only");
    System.out.println("Pulsing shooter: 3s ON, 0.5s OFF");
    System.out.println("==================================");

    // Configure PathPlanner for autonomous path following
    PathPlannerAutoBuilder.configure(driveSubsystem);
    System.out.println("PathPlanner configured for differential drive");

    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem, ballSubsystem));
    
    // Add PathPlanner auto options
    autoChooser.addOption("PathPlanner Test", PathPlannerAutoBuilder.getAutoCommand("TestAuto"));
    autoChooser.addOption("Follow Test Path", PathPlannerAutoBuilder.getFollowPathCommand("TestPath"));
    
    // Add custom autonomous commands
    autoChooser.addOption("Ramp Zigzag Shoot", new RampZigzagShootAuto(driveSubsystem, ballSubsystem));
    autoChooser.addOption("Quick Auto", Autos.quickAuto(driveSubsystem, ballSubsystem));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   */
  private void configureBindings() {
    
    // Initialize camera systems
    LimelightCameraServer.initialize();
    
    // Initialize USB CameraServer for driver camera
    DynamicUSBCameraServer.initialize(0); // Start with device 0
    
    // Configure basic Shuffleboard controls
    SimpleShuffleboardControls.initialize(limelightSubsystem, shooterSubsystem, driveSubsystem);
    
    // === OPERATOR CONTROLLER - SHOOTING CONTROLS ===
    
    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    
    // While the right bumper on operator controller is held, continuous shooter
    operatorController.rightBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.launch(), 
                                          () -> ballSubsystem.stop()));
    
    // Ball ejection with A button (uses only intake motor in reverse)
    operatorController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.ejectIntakeOnly(), () -> ballSubsystem.stop()));
    
    // === PATHPLANNER TEST CONTROLS ===
    
    // Driver controller A button - follow test path
    driverController.a()
        .onTrue(Commands.runOnce(() -> {
            DynamicUSBCameraServer.pauseForAutonomous();
        }).andThen(PathPlannerAutoBuilder.getFollowPathCommand("TestPath"))
        .finallyDo(() -> {
            DynamicUSBCameraServer.resumeFromAutonomous();
        }));
    
    // Driver controller X button - run test autonomous
    driverController.x()
        .onTrue(Commands.runOnce(() -> {
            DynamicUSBCameraServer.pauseForAutonomous();
        }).andThen(PathPlannerAutoBuilder.getAutoCommand("TestAuto"))
        .finallyDo(() -> {
            DynamicUSBCameraServer.resumeFromAutonomous();
        }));
    
    // Operator controller A button - follow square path
    operatorController.a()
        .onTrue(Commands.runOnce(() -> {
            DynamicUSBCameraServer.pauseForAutonomous();
        }).andThen(PathPlannerAutoBuilder.getFollowPathCommand("SquarePath"))
        .finallyDo(() -> {
            DynamicUSBCameraServer.resumeFromAutonomous();
        }));

    // === DRIVER CONTROLLER - MOVEMENT CONTROLS ===
    
    // Driver controller B button toggles movement inversion AND camera port
    driverController.b()
        .onTrue(driveSubsystem.runOnce(() -> {
            // Toggle movement inversion
            driveSubsystem.toggleMovementInversion();
            
            if (driveSubsystem.isMovementInverted()) {
                // Movement inverted - switch to camera port 1
                DynamicUSBCameraServer.switchToDevice(1);
                System.out.println("B button: Movement INVERTED, Camera switched to port 1");
            } else {
                // Movement normal - switch to camera port 0
                DynamicUSBCameraServer.switchToDevice(0);
                System.out.println("B button: Movement NORMAL, Camera switched to port 0");
            }
        }));

    // Switch USB camera with LEFT TRIGGER (operator controller) - simple 2-camera toggle
    operatorController.leftTrigger()
        .onTrue(new InstantCommand(() -> {
            DynamicUSBCameraServer.toggleDevice();  // Simple toggle between 0 and 1
        }));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X-axis is also inverted so a positive value (stick to the right)
    // results in clockwise rotation (front of the robot turning right). Both axes
    // are also scaled down so the rotation is more easily controllable.
    // 
    // Right trigger provides speed boost (up to 50% additional speed)
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            () -> {
              double baseSpeed = -driverController.getLeftY() * DRIVE_SCALING;
              double triggerBoost = driverController.getRightTriggerAxis() * 0.5; // 50% boost
              return baseSpeed + triggerBoost;
            },
            () -> -driverController.getRightX() * ROTATION_SCALING));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
  
  /**
   * Updates all subsystems that need periodic updates.
   */
  public void periodic() {
    // Update camera systems
    LimelightCameraServer.updateStatus();
    
    // Update USB CameraServer status
    DynamicUSBCameraServer.updateStatus();
    
    // Update camera feed broadcaster
    cameraFeedBroadcaster.periodic();
    
    // Update Shuffleboard values
    SimpleShuffleboardControls.updateValues(limelightSubsystem, shooterSubsystem, driveSubsystem);
  }
  

  /**
   * Optional helper used by dynamic button mappers to retrieve commands by name.
   * Currently a stub that returns null for unknown names. Teams should extend
   * this to return actual commands referenced by external configs.
   */
  public Command getCommandForButton(String name) {
    // TODO: map named strings to commands (e.g. "INTAKE" -> ballSubsystem.runEnd(...))
    // Returning null is acceptable; callers already handle missing commands.
    return null;
  }
  
  
  /**
   * Gets camera server information for Shuffleboard.
   * 
   * @return Camera server subsystem
   */
  public SimpleCameraSubsystem getCameraServer() {
    return cameraServerSubsystem;
  }
}
