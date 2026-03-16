// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import static frc.robot.Constants.FuelConstants.*;

/**
 * Fuel subsystem implementation using CTRE VictorSPX motor controllers.
 */
public class FuelSubsystemVictorSpx extends FuelSubsystem {
  private final WPI_VictorSPX feederRoller;
  private final WPI_VictorSPX intakeLauncherRoller;

  /** Creates a new CANFuelSubsystemVictorSpx. */
  public FuelSubsystemVictorSpx() {
    // Create brushed motors for each of the motors on the fuel mechanism
    intakeLauncherRoller = new WPI_VictorSPX(INTAKE_LAUNCHER_MOTOR_ID_SPX);
    feederRoller = new WPI_VictorSPX(FEEDER_MOTOR_ID_SPX);

    // Configure motors
    intakeLauncherRoller.configFactoryDefault();
    feederRoller.configFactoryDefault();

    // Invert the launcher motor so that positive values are used for both intaking and launching
    intakeLauncherRoller.setInverted(true);

    // Enable voltage compensation
    intakeLauncherRoller.configVoltageCompSaturation(12.0);
    intakeLauncherRoller.enableVoltageCompensation(true);
    feederRoller.configVoltageCompSaturation(12.0);
    feederRoller.enableVoltageCompensation(true);

    // Initialize dashboard entries for tuning
    initializeDashboard();
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
