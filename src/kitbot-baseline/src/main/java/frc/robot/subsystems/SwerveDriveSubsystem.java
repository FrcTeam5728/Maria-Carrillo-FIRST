// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Represents a swerve drive style drivetrain.
 */
public class SwerveDriveSubsystem extends DriveSubsystem {
  public static final double kMaxSpeed = 3.0; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  // Module locations relative to robot center
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  // Swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(1, 2, 0, 1, 2, 3);
  private final SwerveModule m_frontRight = new SwerveModule(3, 4, 4, 5, 6, 7);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, 8, 9, 10, 11);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 12, 13, 14, 15);

  // Gyro
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Kinematics and odometry
  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  // Debug counters
  private int debugCounter = 0;

  public SwerveDriveSubsystem() {
    // Initialize parent class components
    initializeGyro();
    initializeOdometry();
    
    m_gyro.reset();
    System.out.println("=== SWERVE DRIVE SUBSYSTEM INITIALIZED ===");
    System.out.println("Max Speed: " + kMaxSpeed + " m/s");
    System.out.println("Max Angular Speed: " + kMaxAngularSpeed + " rad/s");
    System.out.println("Module Locations (meters):");
    System.out.println("  Front Left: " + m_frontLeftLocation.getX() + ", " + m_frontLeftLocation.getY());
    System.out.println("  Front Right: " + m_frontRightLocation.getX() + ", " + m_frontRightLocation.getY());
    System.out.println("  Back Left: " + m_backLeftLocation.getX() + ", " + m_backLeftLocation.getY());
    System.out.println("  Back Right: " + m_backRightLocation.getX() + ", " + m_backRightLocation.getY());
    System.out.println("=======================================");
  }

  @Override
  public void periodic() {
    // Update swerve odometry
    updateOdometry();

    // Debug output
    debugCounter++;
    if (debugCounter % 50 == 0) { // Every 1 second
      Pose2d pose = getPose();
      System.out.println("Swerve Odometry: X=" + String.format("%.3f", pose.getX()) + 
                       ", Y=" + String.format("%.3f", pose.getY()) + 
                       ", Heading=" + String.format("%.1f", pose.getRotation().getDegrees()) +
                       ", Gyro=" + String.format("%.1f", m_gyro.getAngle()));
    }

    // Publish data to SmartDashboard
    Pose2d pose = getPose();
    SmartDashboard.putNumber("Swerve/Odometry/X", pose.getX());
    SmartDashboard.putNumber("Swerve/Odometry/Y", pose.getY());
    SmartDashboard.putNumber("Swerve/Odometry/Heading", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Swerve/Gyro/Angle", m_gyro.getAngle());
    SmartDashboard.putBoolean("Swerve/Gyro/Connected", m_gyro.isConnected());
    
    // Update parent class pose for compatibility
    this.pose = getPose();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  /**
   * Returns the current pose of the robot using swerve odometry.
   *
   * @return The current pose of the robot.
   */
  @Override
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  /**
   * Resets the gyro angle to zero.
   */
  @Override
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Gets the current gyro angle in degrees.
   *
   * @return The gyro angle.
   */
  @Override
  public double getGyroAngle() {
    return m_gyro.getAngle();
  }

  /**
   * Gets the current heading as Rotation2d.
   *
   * @return The current heading.
   */
  @Override
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  /**
   * Arcade drive compatibility method - converts arcade drive inputs to swerve drive.
   * This allows existing code that expects arcade drive to work with swerve drive.
   *
   * @param forward Forward/backward speed (-1.0 to 1.0)
   * @param rotation Rotation speed (-1.0 to 1.0)
   */
  @Override
  public Command driveArcade(DoubleSupplier forward, DoubleSupplier rotation) {
    return this.run(() -> {
      double fwd = forward.getAsDouble() * kMaxSpeed;
      double rot = rotation.getAsDouble() * kMaxAngularSpeed;
      drive(fwd, 0, rot, false); // Drive in robot-relative mode
    });
  }

  /**
   * Creates a command to drive the robot with specified speeds.
   *
   * @param xSpeed Supplier for forward/backward speed
   * @param ySpeed Supplier for left/right speed  
   * @param rot Supplier for rotation speed
   * @param fieldRelative Whether speeds are field relative
   * @return Command to drive the robot
   */
  public Command driveCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot, boolean fieldRelative) {
    return this.run(() -> drive(xSpeed.getAsDouble(), ySpeed.getAsDouble(), rot.getAsDouble(), fieldRelative));
  }

  /**
   * Creates a command to drive the robot in robot-relative mode.
   *
   * @param xSpeed Supplier for forward/backward speed
   * @param ySpeed Supplier for left/right speed
   * @param rot Supplier for rotation speed
   * @return Command to drive the robot
   */
  public Command driveRobotRelative(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot) {
    return driveCommand(xSpeed, ySpeed, rot, false);
  }

  /**
   * Creates a command to drive the robot in field-relative mode.
   *
   * @param xSpeed Supplier for forward/backward speed
   * @param ySpeed Supplier for left/right speed
   * @param rot Supplier for rotation speed
   * @return Command to drive the robot
   */
  public Command driveFieldRelative(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot) {
    return driveCommand(xSpeed, ySpeed, rot, true);
  }

  /**
   * Stops all drive motors.
   */
  public void stop() {
    drive(0, 0, 0, false);
  }

  /**
   * Creates a command to stop the robot.
   *
   * @return Command to stop the robot
   */
  public Command stopCommand() {
    return this.runOnce(this::stop);
  }
}
