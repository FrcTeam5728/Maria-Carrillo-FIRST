// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Configuration for PathPlanner with differential and swerve drive kinematics.
 */
public class PathPlannerConfig {
    
    // Robot physical constants
    public static final double TRACK_WIDTH_METERS = 0.6; // Distance between left and right wheels
    public static final double MAX_VELOCITY_METERS_PER_SEC = 3.0; // Max robot velocity
    public static final double MAX_ACCELERATION_METERS_PER_SEC_SQ = 3.0; // Max robot acceleration
    
    // Robot mass and inertia
    public static final double ROBOT_MASS_KG = 45.0; // Typical FRC robot mass with battery
    public static final double ROBOT_MOI = 6.0; // Moment of inertia in KG*M^2
    
    // Drive system constants
    public static final double WHEEL_RADIUS_METERS = 0.0762; // 3 inch wheels
    public static final double WHEEL_COF = 1.0; // Coefficient of friction
    public static final double DRIVE_CURRENT_LIMIT = 40.0; // Current limit in amps
    public static final int NUM_MOTORS_PER_SIDE = 2; // 2 motors per side for differential drive
    
    // Path following PID constants
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants ROTATION_PID = new PIDConstants(5.0, 0.0, 0.0);
    
    // Holonomic path following settings (for future swerve use)
    public static final double MAX_ANGULAR_VELOCITY_RAD_PER_SEC = Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION_RAD_PER_SEC_SQ = 2 * Math.PI;
    
    /**
     * Creates the robot configuration for PathPlanner with NavX2.
     * 
     * @return RobotConfig configured for differential drive with NavX2
     */
    public static RobotConfig createConfig() {
        try {
            return RobotConfig.fromGUISettings();
        } catch (Exception e) {
            System.err.println("Error loading robot config from GUI settings: " + e.getMessage());
            // Fallback to basic config with correct constructor
            ModuleConfig moduleConfig = new ModuleConfig(
                WHEEL_RADIUS_METERS,
                MAX_VELOCITY_METERS_PER_SEC,
                WHEEL_COF,
                DCMotor.getNEO(1), // Use a single NEO motor as reference
                DRIVE_CURRENT_LIMIT,
                NUM_MOTORS_PER_SIDE
            );
            return new RobotConfig(
                ROBOT_MASS_KG,
                ROBOT_MOI,
                moduleConfig,
                TRACK_WIDTH_METERS
            );
        }
    }
    
    /**
     * Gets the track width for differential drive kinematics.
     * 
     * @return Track width in meters
     */
    public static double getTrackWidth() {
        return TRACK_WIDTH_METERS;
    }
    
    /**
     * Gets the maximum velocity for path planning.
     * 
     * @return Maximum velocity in meters per second
     */
    public static double getMaxVelocity() {
        return MAX_VELOCITY_METERS_PER_SEC;
    }
    
    /**
     * Gets the maximum acceleration for path planning.
     * 
     * @return Maximum acceleration in meters per second squared
     */
    public static double getMaxAcceleration() {
        return MAX_ACCELERATION_METERS_PER_SEC_SQ;
    }
    
    /**
     * Gets the holonomic path follower controller for swerve drive.
     * 
     * @return PPHolonomicDriveController configured for swerve
     */
    public static PPHolonomicDriveController getHolonomicPathFollower() {
        return new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID
        );
    }
}
