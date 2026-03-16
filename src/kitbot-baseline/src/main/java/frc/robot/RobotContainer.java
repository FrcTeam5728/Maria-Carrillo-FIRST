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
import frc.robot.commands.LimelightDiagnosticCommand;
import frc.robot.commands.LimelightTestCommand;
import frc.robot.commands.SimpleAutoShootCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;

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
    // Initialize subsystems (centralized Limelight handles all vision data)
    System.out.println("=== CENTRALIZED LIMELIGHT SYSTEM ===");
    System.out.println("Single source for all Limelight data");
    System.out.println("Pulsing shooter: 3s ON, 0.5s OFF");
    System.out.println("===================================");

    configureBindings();

    // Set the options to show up in the Dashboard for selecting auto modes. If you
    // add additional auto modes you can add additional lines here with
    // autoChooser.addOption
    autoChooser.setDefaultOption("Autonomous", Autos.exampleAuto(driveSubsystem, ballSubsystem));
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
    
    // Limelight diagnostic with BACK button (driver controller)
    driverController.back()
        .onTrue(new LimelightDiagnosticCommand(limelightSubsystem));
    
    // Limelight connection test with START button (driver controller)
    driverController.start()
        .onTrue(new LimelightTestCommand(limelightSubsystem));
    
    // Simple auto-shoot with B button (uses centralized Limelight and pulsing shooter)
    operatorController.b()
        .onTrue(new SimpleAutoShootCommand(driveSubsystem, limelightSubsystem, shooterSubsystem));
    
    // Manual shooter control with Y button
    operatorController.y()
        .whileTrue(shooterSubsystem.runEnd(() -> shooterSubsystem.startContinuous(), 
                                          () -> shooterSubsystem.stop()));
    
    // Pulsing shooter toggle with A button
    operatorController.a()
        .onTrue(shooterSubsystem.runOnce(() -> {
            if (shooterSubsystem.isPulsing()) {
                shooterSubsystem.stop();
                System.out.println("Pulsing shooter stopped");
            } else {
                shooterSubsystem.startPulsing();
                System.out.println("Pulsing shooter started");
            }
        }));
    
    // While the left bumper on operator controller is held, intake Fuel
    operatorController.leftBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.intake(), () -> ballSubsystem.stop()));
    
    // While the right bumper on operator controller is held, manual spin-up
    operatorController.rightBumper()
        .whileTrue(ballSubsystem.runEnd(() -> ballSubsystem.spinUp(), () -> ballSubsystem.stop()));

    // Set the default command for the drive subsystem to the command provided by
    // factory with the values provided by the joystick axes on the driver
    // controller. The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X-axis is also inverted so a positive value (stick to the right)
    // results in clockwise rotation (front of the robot turning right). Both axes
    // are also scaled down so the rotation is more easily controllable.
    driveSubsystem.setDefaultCommand(
        driveSubsystem.driveArcade(
            () -> -driverController.getLeftY() * DRIVE_SCALING,
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
