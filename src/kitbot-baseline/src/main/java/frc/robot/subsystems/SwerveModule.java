// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants.*;

/**
 * Swerve module implementation for a differential drive robot.
 * This module represents one of the four swerve modules that make up the swerve drive.
 */
public class SwerveModule extends SubsystemBase {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private final RelativeEncoder m_driveEncoder;
  private final RelativeEncoder m_turningEncoder;

  private final double m_offset;
  private SwerveModuleState m_state;

  /**
   * Constructs a SwerveModule.
   * 
   * @param driveMotorId   CAN ID of the drive motor
   * @param turningMotorId CAN ID of the turning motor
   * @param driveEncoderId DIO port of the drive encoder
   * @param turningEncoderId DIO port of the turning encoder
   * @param offset         Offset of the module from the center of the robot
   */
  public SwerveModule(int driveMotorId, int turningMotorId, int driveEncoderId, int turningEncoderId, double offset) {
    m_driveMotor = new SparkMax(driveMotorId, MotorType.kBrushed);
    m_turningMotor = new SparkMax(turningMotorId, MotorType.kBrushed);
    m_driveEncoder = new RelativeEncoder(driveEncoderId);
    m_turningEncoder = new RelativeEncoder(turningEncoderId);
    m_offset = offset;

    // Configure the motors
    SparkMaxConfig driveConfig = new SparkMaxConfig();
    driveConfig.smartCurrentLimit(40);
    driveConfig.openLoopRamp(0.1);
    driveConfig.closedLoopRamp(0.1);

    SparkMaxConfig turningConfig = new SparkMaxConfig();
    turningConfig.smartCurrentLimit(30);
    turningConfig.openLoopRamp(0.1);
    turningConfig.closedLoopRamp(0.1);

    m_driveMotor.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turningMotor.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets the desired state of the swerve module.
   * 
   * @param desiredState The desired swerve module state with speeds and angles
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    m_state = desiredState;
  }

  /**
   * Gets the current state of the swerve module.
   * 
   * @return The current swerve module state
   */
  public SwerveModuleState getState() {
    return m_state;
  }

  /**
   * Gets the current drive speed.
   * 
   * @return The current drive speed in meters per second
   */
  public double getDriveSpeed() {
    return m_state.speeds.metersPerSecond;
  }

  /**
   * Gets the current turning speed.
   * 
   * @return The current turning speed in radians per second
   */
  public double getTurningSpeed() {
    return m_state.speeds.omegaRadiansPerSecond;
  }

  /**
   * Gets the current drive angle.
   * 
   * @return The current drive angle in radians
   */
  public double getDriveAngle() {
    return m_state.speeds.angle;
  }

  /**
   * Gets the current turning angle.
   * 
   * @return The current turning angle in radians
   */
  public double getTurningAngle() {
    return m_state.speeds.omega;
  }

  /**
   * Gets the current drive position.
   * 
   * @return The current drive position
   */
  public SwerveModulePosition getPosition() {
    return m_state.positions;
  }

  @Override
  public void periodic() {
    // This method will be called periodically to update the module state
    // Implementation depends on your specific hardware setup
  }
}
