// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Swerve drive subsystem for a differential drive robot.
 * This subsystem controls four swerve modules to achieve omni-directional movement.
 */
public class SwerveDriveSubsystem extends SubsystemBase {
  private final SwerveModule[] m_modules;
  private final SwerveDriveKinematics m_kinematics;
  private final ChassisSpeeds m_chassisSpeeds;
  private final Translation2d m_centerOfRotation;

  /**
   * Constructs a swerve drive subsystem.
   */
  public SwerveDriveSubsystem() {
    // Create swerve modules with offsets from robot center
    double moduleOffset = 0.3; // Distance from center to each module

    m_modules = new SwerveModule[] {
        new SwerveModule(kFrontLeftDriveMotorId, kFrontLeftTurningMotorId, kFrontLeftDriveEncoderId, kFrontLeftTurningEncoderId, moduleOffset),
        new SwerveModule(kFrontRightDriveMotorId, kFrontRightTurningMotorId, kFrontRightDriveEncoderId, kFrontRightTurningEncoderId, moduleOffset),
        new SwerveModule(kBackLeftDriveMotorId, kBackLeftTurningMotorId, kBackLeftDriveEncoderId, kBackLeftTurningEncoderId, moduleOffset),
        new SwerveModule(kBackRightDriveMotorId, kBackRightTurningMotorId, kBackRightDriveEncoderId, kBackRightTurningEncoderId, moduleOffset)
    };

    // Create kinematics object
    m_kinematics = new SwerveDriveKinematics(
        new Translation2d(moduleOffset, moduleOffset),
        new Translation2d(moduleOffset, -moduleOffset),
        new Translation2d(-moduleOffset, moduleOffset),
        new Translation2d(-moduleOffset, -moduleOffset)
    );

    m_centerOfRotation = new Translation2d();

    // Initialize chassis speeds
    m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  }

  /**
   * Sets the swerve module states.
   * 
   * @param speeds The desired chassis speeds (vx, vy, omega)
   * @param gyroAngle The current robot angle
   */
  public void setChassisSpeeds(ChassisSpeeds speeds, Rotation2d gyroAngle) {
    m_chassisSpeeds = speeds;
    
    // Convert chassis speeds to individual module states
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(speeds);
    
    // Set each module's desired state
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setDesiredState(states[i]);
    }
  }

  /**
   * Gets the swerve module states.
   * 
   * @return Array of current swerve module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getState();
    }
    return states;
  }

  /**
   * Gets the swerve kinematics object.
   * 
   * @return The swerve drive kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  /**
   * Gets the chassis speeds.
   * 
   * @return The current chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return m_chassisSpeeds;
  }

  /**
   * Gets the center of rotation.
   * 
   * @return The center of rotation for the swerve drive
   */
  public Translation2d getCenterOfRotation() {
    return m_centerOfRotation;
  }

  @Override
  public void periodic() {
    // This method will be called periodically to update the swerve drive
    // Implementation depends on your specific hardware setup
  }
}
