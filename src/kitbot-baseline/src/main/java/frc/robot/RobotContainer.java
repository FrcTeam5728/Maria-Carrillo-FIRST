// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.OperatorConstants.*;
import static frc.robot.Constants.FuelConstants.*;
import frc.robot.commands.Autos;
import frc.robot.commands.CameraDetectionCommand;
import frc.robot.commands.LimelightDiagnosticCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.PathPlannerSubsystem;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.commands.CameraControlCommand;
import frc.robot.commands.FieldInfoCommand;
import frc.robot.utils.CameraFeedStreamer;
import frc.robot.utils.TeleopControlModifier;

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
  private final CameraFeedStreamer cameraStreamer;
  private final PathPlannerSubsystem pathPlannerSubsystem = new PathPlannerSubsystem(
      driveSubsystem, 
      Constants.DriveConstants.kTrackWidthMeters
  );
  private final AprilTagSubsystem aprilTagSubsystem;

  // The driver's controller
  private final CommandXboxController driverController = new CommandXboxController(
      DRIVER_CONTROLLER_PORT);

  // The operator's controller
  private final CommandXboxController operatorController = new CommandXboxController(
      OPERATOR_CONTROLLER_PORT);

  // Control modifier for applying constraints
  private final TeleopControlModifier controlModifier;

  // The autonomous chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize AprilTag subsystem
    aprilTagSubsystem = RobotSubsystemFactory.createAprilTagSubsystem();
    // Initialize control modifier with AprilTag subsystem
    this.controlModifier = new TeleopControlModifier(aprilTagSubsystem);
    
    configureBindings();
    
    // Configure autonomous options
    configureAutonomous();
    
    // Start with camera streaming by default if the native libraries are available.
    CameraFeedStreamer tmpCamera = null;
    try {
      tmpCamera = CameraFeedStreamer.getInstance();
      if (tmpCamera != null) {
        System.out.println("CameraFeedStreamer initialized successfully");
        if (!tmpCamera.isOpenCvAvailable()) {
          System.out.println("Camera will work without OpenCV image processing");
        }
      }
    } catch (UnsatisfiedLinkError ule) {
      System.err.println("Camera native library not available: " + ule.getMessage());
    } catch (Exception e) {
      System.err.println("Failed to initialize CameraFeedStreamer: " + e.getMessage());
    }
    this.cameraStreamer = tmpCamera;
    if (this.cameraStreamer != null) {
      try {
        this.cameraStreamer.startStreaming();
      } catch (Exception e) {
        System.err.println("Camera streamer failed to start: " + e.getMessage());
      }
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
   * for {@link CommandXboxController Xbox}/
   * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  /**
   * Configures the autonomous chooser with available autonomous routines.
   */
  private void configureAutonomous() {
    // Add path planner autos to the chooser
    autoChooser.setDefaultOption("Example Auto", Autos.exampleAuto(driveSubsystem, ballSubsystem));
    
    // Add PathPlanner autos - these will be automatically discovered from the deploy/pathplanner/autos directory
    // The names here should match the filenames in the autos directory (without .auto)
    Command examplePathCmd = pathPlannerSubsystem.followPath("Example Path");
    if (examplePathCmd != null) {
      autoChooser.addOption("Example Path", examplePathCmd);
    } else {
      System.err.println("PathPlanner: 'Example Path' not available (followPath returned null)");
    }
    
    // Add more path planner autos as needed
    // autoChooser.addOption("Another Path", pathPlannerSubsystem.followPath("Another Path"));
  }

  /**
   * Configures the button bindings for the robot.
   */
  private void configureBindings() {

    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    // While the right bumper on the operator controller is held, spin up for 1
    // second, then launch fuel. When the button is released, stop.
    operatorController.rightBumper()
        .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(ballSubsystem.launchCommand())
            .finallyDo(() -> ballSubsystem.stop()));
    // While the A button is held on the operator controller, eject fuel back out
    // the intake
    operatorController.a()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.eject(), () -> ballSubsystem.stop()));
    
    // Simple shooting with B button
    operatorController.b()
        .whileTrue(ballSubsystem.spinUpCommand().withTimeout(SPIN_UP_SECONDS)
            .andThen(ballSubsystem.launchCommand())
            .finallyDo(() -> ballSubsystem.stop()));
    
    // Camera streaming controls (driver controller) - only if camera is available
    if (cameraStreamer != null) {
      driverController.povUp()
          .onTrue(new CameraControlCommand(cameraStreamer, true)); // Enable camera
      
      driverController.povDown()
          .onTrue(new CameraControlCommand(cameraStreamer, false)); // Disable camera
    }
    
    // Camera quality controls (operator controller)
    operatorController.leftTrigger()
        .whileTrue(ballSubsystem.runOnce(() -> {
          if (cameraStreamer != null) {
            cameraStreamer.setCompressionLevel(30);
          } else {
            System.err.println("Camera not available - cannot set compression level");
          }
        })); // High quality
    
    operatorController.rightTrigger()
        .whileTrue(ballSubsystem.runOnce(() -> {
          if (cameraStreamer != null) {
            cameraStreamer.setCompressionLevel(70);
          } else {
            System.err.println("Camera not available - cannot set compression level");
          }
        })); // Low quality
    
    // Field info command (driver controller)
    driverController.start()
        .onTrue(new FieldInfoCommand()); // Show field info on start press
    
    // Limelight diagnostic command (driver controller)
    driverController.back()
        .onTrue(new LimelightDiagnosticCommand(aprilTagSubsystem)); // Run Limelight diagnostic on back press
    
    // Camera detection command (driver controller)
    driverController.leftStick()
        .onTrue(new CameraDetectionCommand()); // Run camera detection when left stick is pressed

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X-axis is also inverted so a positive value (stick to the right)
    // results in clockwise rotation (front of the robot turning right). Both axes
    // are also scaled down so the rotation is more easily controllable.
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            () -> {
              double forward = -driverController.getLeftY() * DRIVE_SCALING;
              double rotation = -driverController.getRightX() * ROTATION_SCALING;
              
              // Apply constraints to the control inputs
              double[] modifiedControls = controlModifier.applyConstraints(forward, rotation);
              return modifiedControls[0]; // Return modified forward
            },
            () -> {
              double forward = -driverController.getLeftY() * DRIVE_SCALING;
              double rotation = -driverController.getRightX() * ROTATION_SCALING;
              
              // Apply constraints to the control inputs
              double[] modifiedControls = controlModifier.applyConstraints(forward, rotation);
              return modifiedControls[1]; // Return modified rotation
            }));
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
   * Optional helper used by dynamic button mappers to retrieve commands by name.
   * Currently a stub that returns null for unknown names. Teams should extend
   * this to return actual commands referenced by external configs.
   */
  public Command getCommandForButton(String name) {
    // TODO: map named strings to commands (e.g. "INTAKE" -> ballSubsystem.runEnd(...))
    // Returning null is acceptable; callers already handle missing commands.
    return null;
  }
}
