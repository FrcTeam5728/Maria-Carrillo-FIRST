// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class FactoryConstants {
    // Drive subsystem type selection: "SPARKMAX" or "VICTORSPX"
    public static final String DRIVE_SUBSYSTEM_TYPE = "SPARKMAX";

    // Fuel subsystem type selection: "SPARKMAX" or "VICTORSPX"
    public static final String FUEL_SUBSYSTEM_TYPE = "SPARKMAX";
  }

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID_SPARKMAX = 21;
    public static final int LEFT_FOLLOWER_ID_SPARKMAX = 22;
    public static final int RIGHT_LEADER_ID_SPARKMAX = 23;
    public static final int RIGHT_FOLLOWER_ID_SPARKMAX = 24;

    public static final int LEFT_LEADER_ID_SPX = 1;
    public static final int LEFT_FOLLOWER_ID_SPX = 2;
    public static final int RIGHT_LEADER_ID_SPX = 3;
    public static final int RIGHT_FOLLOWER_ID_SPX = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;

    // Physical properties of the robot
    public static final double kWheelDiameterMeters = 0.0951; // 6 inches in meters
    public static final double kGearRatio = 8.4; // Example: 10.71:1 gear ratio
    public static final double kTrackWidthMeters = 0.69; // Example: 27.2 inches in meters
    
    // Path following constants
    public static final double kMaxSpeedMetersPerSecond = 3.0; // Max speed of the robot in m/s
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0; // Max acceleration in m/s²
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // Max angular speed in rad/s
    
    // PID constants for path following
    public static final double kPDriveVel = 8.5;
    public static final double kIDriveVel = 0.0;
    public static final double kDDriveVel = 0.0;
    
    // Feedforward constants for path following
    public static final double ksVolts = 0.1; // Static voltage gain
    public static final double kvVoltSecondsPerMeter = 1.5; // Velocity gain
    public static final double kaVoltSecondsSquaredPerMeter = 0.2; // Acceleration gain
    
    // Drive kinematics
    public static final double kWheelBaseMeters = kTrackWidthMeters;
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID_SPARKMAX = 21;
    public static final int INTAKE_LAUNCHER_MOTOR_ID_SPARKMAX = 21;
    public static final int SHOOTER_LEFT_MOTOR_ID_SPARKMAX = 21;  // Left shooter (CAN ID 21)
    public static final int SHOOTER_RIGHT_MOTOR_ID_SPARKMAX = 23; // Right shooter (CAN ID 23)

    public static final int FEEDER_MOTOR_ID_SPX = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID_SPX = 5;
    public static final int SHOOTER_LEFT_MOTOR_ID_SPX = 31;   // Left shooter (CAN ID 21)
    public static final int SHOOTER_RIGHT_MOTOR_ID_SPX = 34;  // Right shooter (CAN ID 23)

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 40;
    
    // Shooting calculation constants
    public static final double SHOOTER_SPINUP_TIME_SECONDS = 5.0; // Time to reach target RPM (calibrated)
    public static final double MAX_SHOOTING_DISTANCE_FEET = 2.5; // Maximum effective shooting distance
    public static final double MAX_SHOOTING_DISTANCE_METERS = MAX_SHOOTING_DISTANCE_FEET * 0.3048; // Converted to meters
    public static final double CALIBRATION_VOLTAGE = 11.5; // Voltage used for calibration
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 40;
    public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 40;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = 9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = -6;
    public static final double SPIN_UP_SECONDS = 1.0;
  }

  public static final class OperatorConstants {
    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 0; // Both driver and operator use same controller;

    // This value is multiplied by the joystick value when driving the robot to
    // help avoid driving and turning too fast and being difficult to control
    public static final double DRIVE_SCALING = .7;
    public static final double ROTATION_SCALING = .8;
  }
}
