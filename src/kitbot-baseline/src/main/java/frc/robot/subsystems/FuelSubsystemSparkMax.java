// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystemSparkMax extends FuelSubsystem {
  private final SparkMax feederRoller;
  private final SparkMax intakeLauncherRoller;
  private final SparkMax leftShooter;
  private final SparkMax rightShooter;
  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;
  private boolean isLaunching = false;

  /** Creates a new CANBallSubsystem. */
  public FuelSubsystemSparkMax() {
    // create brushed motors for each of the motors on the launcher mechanism
    intakeLauncherRoller = new SparkMax(INTAKE_LAUNCHER_MOTOR_ID_SPARKMAX, MotorType.kBrushed);
    feederRoller = new SparkMax(FEEDER_MOTOR_ID_SPARKMAX, MotorType.kBrushed);
    leftShooter = new SparkMax(SHOOTER_LEFT_MOTOR_ID_SPARKMAX, MotorType.kBrushed);
    rightShooter = new SparkMax(SHOOTER_RIGHT_MOTOR_ID_SPARKMAX, MotorType.kBrushed);

    // Get encoders from shooter motors
    leftEncoder = leftShooter.getEncoder();
    rightEncoder = rightShooter.getEncoder();

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

    // Configure shooter motors
    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.smartCurrentLimit(SHOOTER_MOTOR_CURRENT_LIMIT);
    leftShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightShooter.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
    // Convert RPM to velocity for SparkMax (assuming 6000 RPM = 1.0 velocity)
    double velocity = rpm / 6000.0;
    leftShooter.set(velocity);
    rightShooter.set(velocity);
  }

  @Override
  public double getShooterRPM() {
    // Get average RPM from both shooter encoders
    double leftRPM = leftEncoder.getVelocity() * 6000.0;
    double rightRPM = rightEncoder.getVelocity() * 6000.0;
    return (leftRPM + rightRPM) / 2.0;
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
