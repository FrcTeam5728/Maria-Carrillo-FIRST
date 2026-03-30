// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.swerveUtil.COTSNeoSwerveConstants;
import frc.lib.util.swerveUtil.SwerveModuleConstants;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Num;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

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
    // Drive subsystem type selection: "SPARKMAX", "VICTORSPX", or "SWERVE"
    public static final String DRIVE_SUBSYSTEM_TYPE = "SWERVE";

    // Fuel subsystem type selection: "SPARKMAX" or "VICTORSPX"
    public static final String FUEL_SUBSYSTEM_TYPE = "SPARKMAX";
  }

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID_SPARKMAX = 33;
    public static final int LEFT_FOLLOWER_ID_SPARKMAX = 32;
    public static final int RIGHT_LEADER_ID_SPARKMAX = 31;
    public static final int RIGHT_FOLLOWER_ID_SPARKMAX = 34;

    public static final int LEFT_LEADER_ID_SPX = 1;
    public static final int LEFT_FOLLOWER_ID_SPX = 2;
    public static final int RIGHT_LEADER_ID_SPX = 3;
    public static final int RIGHT_FOLLOWER_ID_SPX = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 40;
  }

  public static final class Swerve {
    public static final COTSNeoSwerveConstants chosenModule = 
      COTSNeoSwerveConstants.SDSMK4i(COTSNeoSwerveConstants.driveGearRatios.SDSMK4i_L1);
    
    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(28);
    public static final double wheelBase = Units.inchesToMeters(28);
    public static final double wheelCircumference = chosenModule.wheelCircumference;
    public static final double driveRevToMeters = wheelCircumference / (chosenModule.driveGearRatio);
    public static final double driveRpmToMetersPerSecond = driveRevToMeters/60;
    
    /* Swerve Kinematics */
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
    );
    
    /* Module Gear Ratios */
    public static final double driveGearRatio = chosenModule.driveGearRatio;
    public static final double angleGearRatio = chosenModule.angleGearRatio;
    
    /* Motor Inverts */
    public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
    public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;
    
    /* Angle Encoder Invert */
    public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;
    
    /* Swerve Current Limiting */
    public static final int angleCurrentLimit = 25;
    public static final int angleCurrentThreshold = 40;
    public static final double angleCurrentThresholdTime = 0.1;
    public static final boolean angleEnableCurrentLimit = true;
    
    public static final int driveCurrentLimit = 40;
    public static final int driveCurrentThreshold = 60;
    public static final double driveCurrentThresholdTime = 0.1;
    public static final boolean driveEnableCurrentLimit = true;
    
    /* These values are used by the drive Motor to ramp in open loop and closed loop driving. */
    public static final double openLoopRamp = 0.2;
    public static final double closedLoopRamp = 0.0;
    
    /* Angle Motor PID Values */
    public static final double angleKP = chosenModule.angleKP;
    public static final double angleKI = chosenModule.angleKI;
    public static final double angleKD = chosenModule.angleKD;
    public static final double angleKF = chosenModule.angleKF;
    
    /* Drive Motor PID Values */
    public static final double driveKP = 0.012;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKF = 0.0;
    
    /* Heading PID Values */
    public static final double HeadingKP = 0.5;
    public static final double HeadingKI = 0.0;
    public static final double HeadingKD = 0;
    public static final double HeadingTolerence = 0;
    
    //Motor power gain
    public static final double drivePower = 1;
    public static final double anglePower = .9;
    
    /* Drive Motor Characterization Values from SysID */
    public static final double driveKS = (0.32);
    public static final double driveKV = (1.51);
    public static final double driveKA = (0.27);
    
    /* Swerve Profiling Values */
    /** Meters per Second */
    public static final double maxSpeed = 4.5;
    /** Radians per Second */
    public static final double maxAngularVelocity = 10.0;
    
    /* Neutral Modes */
    // public static final SparkLowLevel.IdleMode angleNuetralMode = SparkLowLevel.IdleMode.kCoast;
    // public static final SparkLowLevel.IdleMode driveNuetralMode = SparkLowLevel.IdleMode.kBrake;
    
    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 7;
      public static final int canCoderID = 9;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.05002);
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 2;
      public static final int angleMotorID = 1;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.037598);
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 10;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.499268);
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
    
    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 4;
      public static final int angleMotorID = 3;
      public static final int canCoderID = 12;
      public static final Rotation2d angleOffset = Rotation2d.fromRotations(-0.389404);
      public static final SwerveModuleConstants constants = 
        new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }
  
  public static final class PoseEstimator {
    public static final Matrix<N3, N1> stateStdDevs = new Matrix<>(N3.instance, N1.instance);
    public static final Matrix<N3, N1> visionStdDevs = new Matrix<>(N3.instance, N1.instance);
    
    static {
      stateStdDevs.set(0, 0, 0.1);
      stateStdDevs.set(1, 0, 0.1);
      stateStdDevs.set(2, 0, 0.1);
      
      visionStdDevs.set(0, 0, 0.5);
      visionStdDevs.set(1, 0, 0.5);
      visionStdDevs.set(2, 0, 0.5);
    }
  }
  
  public static final class AutoConstants {
    public static final double ROBOT_MASS_KG = 54.0;
    public static final double ROBOT_MOI = 6.78;
    public static final SwerveModuleConstants[] moduleConfig = 
      new SwerveModuleConstants[] {
        Constants.Swerve.Mod0.constants,
        Constants.Swerve.Mod1.constants,
        Constants.Swerve.Mod2.constants,
        Constants.Swerve.Mod3.constants
      };
    
    public static final double translationPID[] = {3.0, 0.0, 0.0};
    public static final double rotationPID[] = {3.0, 0.0, 0.0};
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int FEEDER_MOTOR_ID_SPARKMAX = 22;
    public static final int INTAKE_LAUNCHER_MOTOR_ID_SPARKMAX = 21;

    public static final int FEEDER_MOTOR_ID_SPX = 6;
    public static final int INTAKE_LAUNCHER_MOTOR_ID_SPX = 5;

    // Current limit and nominal voltage for fuel mechanism motors.
    public static final int FEEDER_MOTOR_CURRENT_LIMIT = 40;
    public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 40;

    // Voltage values for various fuel operations. These values may need to be tuned
    // based on exact robot construction.
    // See the Software Guide for tuning information
    public static final double INTAKING_FEEDER_VOLTAGE = -12;
    public static final double INTAKING_INTAKE_VOLTAGE = 10;
    public static final double LAUNCHING_FEEDER_VOLTAGE = -9;
    public static final double LAUNCHING_LAUNCHER_VOLTAGE = 10.6;
    public static final double SPIN_UP_FEEDER_VOLTAGE = 6;
    public static final double SPIN_UP_SECONDS = 1;
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
