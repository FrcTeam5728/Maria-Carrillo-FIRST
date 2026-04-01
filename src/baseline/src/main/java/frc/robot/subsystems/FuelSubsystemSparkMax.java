// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystemSparkMax extends FuelSubsystem {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;

  /** Creates a new CANBallSubsystem. */
  public FuelSubsystemSparkMax() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID_SPARKMAX, MotorType.kBrushed);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID_SPARKMAX, MotorType.kBrushed);

    // Initialize dashboard entries for tuning
    initializeDashboard();

    // create the configuration for the feeder roller, set a current limit and apply
    // the config to the controller
    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);
    feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // create the configuration for the launcher roller, set a current limit, set
    // the motor to inverted so that positive values are used for both intaking and
    // launching, and apply the config to the controller
    SparkMaxConfig launcherConfig = new SparkMaxConfig();
    launcherConfig.inverted(true);
    launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
    intakeLauncherRoller.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  protected void setFeederVoltage(double voltage) {
    feederRoller.setVoltage(voltage);
  }

  @Override
  protected void setIntakeLauncherVoltage(double voltage) {
    intakeLauncherRoller.setVoltage(voltage);
  }
}
