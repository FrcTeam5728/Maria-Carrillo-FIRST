// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.config.ShuffleboardManager;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PulsingShooterSubsystem;
import frc.robot.subsystems.SimpleCameraSubsystem;
import frc.robot.utils.CameraFeedBroadcaster;
import frc.robot.utils.FieldPositionSystem;
import frc.robot.utils.ShootingPositionManager;
import frc.robot.subsystems.SwerveDriveSubsystem;

import static frc.robot.Constants.DriveConstants.*;
import static frc.robot.Constants.OIConstants.*;

/**
 * This class is where the bulk of the robot components are declared.
 * This class declares subsystems and command objects for the entire robot.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SwerveDriveSubsystem m_swerveDrive;
  private final FuelSubsystem m_fuelSubsystem;
  private final LimelightSubsystem m_limelightSubsystem;
  private final PulsingShooterSubsystem m_shooterSubsystem;
  private final SimpleCameraSubsystem m_cameraSubsystem;
  private final CameraFeedBroadcaster cameraFeedBroadcaster;
  private final FieldPositionSystem fieldPositionSystem;
  private final ShootingPositionManager shootingPositionManager;

  // The operator's controller
  private final CommandXboxController m_driverController;
  private final CommandXboxController m_operatorController;

  // Shuffleboard manager for displaying robot data
  private final ShuffleboardManager shuffleboardManager = new ShuffleboardManager();

  // Autonomous command chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Initialize subsystems
    m_swerveDrive = new SwerveDriveSubsystem();
    m_fuelSubsystem = new FuelSubsystem();
    m_limelightSubsystem = new LimelightSubsystem();
    m_shooterSubsystem = new PulsingShooterSubsystem();
    m_cameraSubsystem = new SimpleCameraSubsystem();
    cameraFeedBroadcaster = new CameraFeedBroadcaster();
    fieldPositionSystem = new FieldPositionSystem();
    shootingPositionManager = new ShootingPositionManager();

    // Initialize operator controllers
    m_driverController = new CommandXboxController(kDriverControllerPort);
    m_operatorController = new CommandXboxController(kOperatorControllerPort);

    // Initialize Shuffleboard manager
    shuffleboardManager = new ShuffleboardManager();

    // Configure button bindings
    configureBindings();

    // Initialize autonomous chooser
    autoChooser.setDefaultOption("Swerve Auto", Autos.swerveAuto(m_swerveDrive, m_fuelSubsystem));
  }

  /**
   * Gets the drive subsystem.
   * 
   * @return The swerve drive subsystem
   */
  public SwerveDriveSubsystem getSwerveDrive() {
    return m_swerveDrive;
  }

  /**
   * Gets the fuel subsystem.
   * 
   * @return The fuel subsystem
   */
  public FuelSubsystem getFuelSubsystem() {
    return m_fuelSubsystem;
  }

  /**
   * Gets the limelight subsystem.
   * 
   * @return The limelight subsystem
   */
  public LimelightSubsystem getLimelightSubsystem() {
    return m_limelightSubsystem;
  }

  /**
   * Gets the shooter subsystem.
   * 
   * @return The shooter subsystem
   */
  public PulsingShooterSubsystem getShooterSubsystem() {
    return m_shooterSubsystem;
  }

  /**
   * Gets the camera subsystem.
   * 
   * @return The camera subsystem
   */
  public SimpleCameraSubsystem getCameraSubsystem() {
    return m_cameraSubsystem;
  }

  /**
   * Gets the field position system.
   * 
   * @return The field position system
   */
  public FieldPositionSystem getFieldPositionSystem() {
    return fieldPositionSystem;
  }

  /**
   * Gets the shooting position manager.
   * 
   * @return The shooting position manager
   */
  public ShootingPositionManager getShootingPositionManager() {
    return shootingPositionManager;
  }

  /**
   * Gets the autonomous command selected.
   * 
   * @return The selected autonomous command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Configures the button bindings.
   */
  private void configureBindings() {
    // Driver controller bindings
    m_swerveDrive.setDefaultCommand(
        m_swerveDrive::driveCommand(
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX(),
            () -> -m_driverController.getRightX()
        )
    );

    // Operator controller bindings
    m_operatorController.a().whileTrue(m_shooterSubsystem::intake).onFalse(m_shooterSubsystem::stop);
    m_operatorController.b().whileTrue(m_shooterSubsystem::launch).onFalse(m_shooterSubsystem::stop);
    m_operatorController.x().whileTrue(m_shooterSubsystem::intake).onFalse(m_shooterSubsystem::stop);
    m_operatorController.y().whileTrue(m_shooterSubsystem::launch).onFalse(m_shooterSubsystem::stop);

    // Update Shuffleboard with subsystem data
    shuffleboardManager.updateAllWidgets(
        m_cameraSubsystem,
        m_limelightSubsystem,
        fieldPositionSystem,
        shootingPositionManager,
        m_swerveDrive
    );
  }
}
