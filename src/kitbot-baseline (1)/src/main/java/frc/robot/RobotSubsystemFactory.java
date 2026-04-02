package frc.robot;

import frc.robot.subsystems.DriveSubsystemSparkMax;
import frc.robot.subsystems.DriveSubsystemVictorSpx;
import frc.robot.subsystems.FuelSubsystemSparkMax;
import frc.robot.subsystems.FuelSubsystemVictorSpx;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import static frc.robot.Constants.FactoryConstants.*;

/**
 * Factory class for creating robot subsystems based on configuration constants.
 * This allows easy switching between different motor controller implementations
 * without changing code throughout the robot.
 */
public class RobotSubsystemFactory {
    /**
     * Creates a drive subsystem based on the DRIVE_SUBSYSTEM_TYPE constant.
     * 
     * @return A DriveSubsystem implementation (either SparkMax or VictorSPX based)
     * @throws IllegalArgumentException if the subsystem type is not recognized
     */
    public static DriveSubsystem createDriveSubsystem() {
        switch (DRIVE_SUBSYSTEM_TYPE.toUpperCase()) {
            case "SPARKMAX":
                return new DriveSubsystemSparkMax();
            case "VICTORSPX":
                return new DriveSubsystemVictorSpx();
            default:
                throw new IllegalArgumentException(
                    "Unknown drive subsystem type: " + DRIVE_SUBSYSTEM_TYPE + 
                    ". Must be 'SPARKMAX' or 'VICTORSPX'");
        }
    }

    /**
     * Creates a fuel subsystem based on the FUEL_SUBSYSTEM_TYPE constant.
     * 
     * @return A FuelSubsystem implementation (either SparkMax or VictorSFX based)
     * @throws IllegalArgumentException if the subsystem type is not recognized
     */
    public static FuelSubsystem createFuelSubsystem() {
        switch (FUEL_SUBSYSTEM_TYPE.toUpperCase()) {
            case "SPARKMAX":
                return new FuelSubsystemSparkMax();
            case "VICTORSPX":
                return new FuelSubsystemVictorSpx();
            default:
                throw new IllegalArgumentException(
                    "Unknown fuel subsystem type: " + FUEL_SUBSYSTEM_TYPE + 
                    ". Must be 'SPARKMAX' or 'VICTORSPX'");
        }
    }
}
