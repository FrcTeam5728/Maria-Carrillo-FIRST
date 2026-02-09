// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import static frc.robot.Constants.DriveConstants.*;

public class DriveSubsystemVictorSpx extends DriveSubsystem {
  private final WPI_VictorSPX leftLeader;
  private final WPI_VictorSPX leftFollower;
  private final WPI_VictorSPX rightLeader;
  private final WPI_VictorSPX rightFollower;

  private void configureDriveMotorControllerSpx(WPI_VictorSPX motor, boolean inverted) {
    motor.configFactoryDefault();
    
    // Configure inversion
    motor.setInverted(inverted);

    // Enable voltage compensation
    motor.configVoltageCompSaturation(12.0);
    motor.enableVoltageCompensation(true);
  } 

  public DriveSubsystemVictorSpx() {
    // create brushed motors for drive
    leftLeader = new WPI_VictorSPX(LEFT_LEADER_ID_SPX);
    leftFollower = new WPI_VictorSPX(LEFT_FOLLOWER_ID_SPX);
    rightLeader = new WPI_VictorSPX(RIGHT_LEADER_ID_SPX);
    rightFollower = new WPI_VictorSPX(RIGHT_FOLLOWER_ID_SPX);

    // Configure motor controllers
    configureDriveMotorControllerSpx(leftLeader, false);  // Left side inverted
    configureDriveMotorControllerSpx(leftFollower, false);
    configureDriveMotorControllerSpx(rightLeader, true);
    configureDriveMotorControllerSpx(rightFollower, true);

    leftFollower.follow(leftLeader);
    rightFollower.follow(rightLeader);

    // set up differential drive class
    drive = new DifferentialDrive(leftLeader, rightLeader);

    // Set watchdog timer to 2.0 seconds (TODO: adjust as needed)
    //drive.setExpiration(2.0);
    //drive.setSafetyEnabled(true);
  }

  @Override
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    // Scale volts to percent (12V -> 1.0) and drive using DifferentialDrive
    drive.tankDrive(leftVolts / 12.0, rightVolts / 12.0);
    drive.feed();
  }

  @Override
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // No encoders implemented in this example; return zeros. Replace with
    // actual encoder readings for real robots.
    return new DifferentialDriveWheelSpeeds(0.0, 0.0);
  }

}
