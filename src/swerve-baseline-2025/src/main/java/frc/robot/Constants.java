// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(2.5);
  // Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;

    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }

  public static final class LimelightConstants
  {
    // Limelight configuration
    public static final String LIMELIGHT_NAME = "limelight";
    public static final double LIMELIGHT_MOUNT_HEIGHT = 0.5; // meters
    public static final double LIMELIGHT_ANGLE = 15.0; // degrees from horizontal
    public static final double TARGET_HEIGHT = 0.0; // meters (adjust based on actual target)
    
    // Pipeline constants
    public static final int DEFAULT_PIPELINE = 0;
    public static final int RETROREFLECTIVE_PIPELINE = 0;
    public static final int APRILTAG_PIPELINE = 1;
    
    // LED modes
    public static final int LED_OFF = 0;
    public static final int LED_BLINK = 1;
    public static final int LED_ON = 2;
    public static final int LED_USE_PIPELINE = 3;
    
    // Vision targeting constants
    public static final double TARGETING_KP = 0.02; // Proportional gain for targeting
    public static final double TARGETING_KD = 0.0; // Derivative gain for targeting
    public static final double MAX_TARGETING_SPEED = 2.0; // m/s
    public static final double TARGETING_TOLERANCE = 1.0; // degrees
  }

  public static final class PathPlannerConstants
  {
    // Path following constants
    public static final double MAX_VELOCITY = 4.0; // m/s
    public static final double MAX_ACCELERATION = 3.0; // m/s^2
    public static final double MAX_ANGULAR_VELOCITY = Math.PI; // rad/s
    public static final double MAX_ANGULAR_ACCELERATION = Math.PI; // rad/s^2
    
    // PID constants for path following
    public static final double TRANSLATION_KP = 5.0;
    public static final double TRANSLATION_KI = 0.0;
    public static final double TRANSLATION_KD = 0.0;
    
    public static final double ROTATION_KP = 5.0;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;
    
    // Replanning configuration
    public static final boolean ENABLE_DYNAMIC_REPLANNING = true;
    public static final double REPLANNING_INTERVAL = 0.1; // seconds
  }
}