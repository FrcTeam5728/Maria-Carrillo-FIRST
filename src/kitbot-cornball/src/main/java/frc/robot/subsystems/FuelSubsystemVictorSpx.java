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
  private final WPI_VictorSPX leftShooter;
  private final WPI_VictorSPX rightShooter;
  private boolean isLaunching = false;

  /** Creates a new CANFuelSubsystemVictorSpx. */
  public FuelSubsystemVictorSpx() {
    // Create brushed motors for each of the motors on the fuel mechanism
    intakeLauncherRoller = new WPI_VictorSPX(INTAKE_LAUNCHER_MOTOR_ID_SPX);
    feederRoller = new WPI_VictorSPX(FEEDER_MOTOR_ID_SPX);
    leftShooter = new WPI_VictorSPX(SHOOTER_LEFT_MOTOR_ID_SPX);
    rightShooter = new WPI_VictorSPX(SHOOTER_RIGHT_MOTOR_ID_SPX);

    // Configure motors
    intakeLauncherRoller.configFactoryDefault();
    feederRoller.configFactoryDefault();
    leftShooter.configFactoryDefault();
    rightShooter.configFactoryDefault();

    // Invert the launcher motor so that positive values are used for both intaking and launching
    intakeLauncherRoller.setInverted(true);

    // Enable voltage compensation for all motors
    intakeLauncherRoller.configVoltageCompSaturation(12.0);
    intakeLauncherRoller.enableVoltageCompensation(true);
    feederRoller.configVoltageCompSaturation(12.0);
    feederRoller.enableVoltageCompensation(true);
    leftShooter.configVoltageCompSaturation(12.0);
    leftShooter.enableVoltageCompensation(true);
    rightShooter.configVoltageCompSaturation(12.0);
    rightShooter.enableVoltageCompensation(true);

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

  @Override
  public void setShooterRPM(double rpm) {
    // Convert RPM to voltage for VictorSPX (approximate conversion)
    // This is a simplified approach - you may need to tune the scaling factor
    double voltage = (rpm / 6000.0) * 12.0; // Scale to 12V max
    voltage = Math.max(-12.0, Math.min(12.0, voltage)); // Clamp to voltage limits
    leftShooter.setVoltage(voltage);
    rightShooter.setVoltage(voltage);
  }

  @Override
  public double getShooterRPM() {
    // VictorSPX doesn't have built-in encoders, so we'll estimate based on voltage
    // For accurate RPM, you would need external encoders
    double leftVoltage = leftShooter.getMotorOutputVoltage();
    double rightVoltage = rightShooter.getMotorOutputVoltage();
    double avgVoltage = (leftVoltage + rightVoltage) / 2.0;
    
    // Convert voltage back to RPM (reverse of the conversion in setShooterRPM)
    double rpm = (avgVoltage / 12.0) * 6000.0;
    return rpm;
  }

  @Override
  public boolean isLaunching() {
    return isLaunching;
  }

  @Override
  public void launch() {
    super.launch();
    isLaunching = true;
  }

  @Override
  public void stop() {
    super.stop();
    isLaunching = false;
  }
}
