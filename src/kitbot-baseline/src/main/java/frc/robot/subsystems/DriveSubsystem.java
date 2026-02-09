package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Abstract base class for drive subsystems.
 * Provides a common interface and shared implementation for different drivetrain motor controller implementations.
 * 
 * This class manages the DifferentialDrive instance and provides a default implementation
 * of driveArcade() that subclasses can override if needed.
 */
public abstract class DriveSubsystem extends SubsystemBase {
    /**
     * The differential drive instance shared by all drive subsystems.
     * Protected so subclasses can initialize it in their constructors.
     */
    protected DifferentialDrive drive;

    /**
     * Creates an arcade drive command.
     * Provides a default implementation using the drive field.
     * Subclasses should initialize the drive field in their constructor before this is called.
     * 
     * @param forward DoubleSupplier for forward/backward speed
     * @param rotation DoubleSupplier for rotation speed
     * @return Command to drive the robot
     */
    public Command driveArcade(DoubleSupplier forward, DoubleSupplier rotation) {
        return this.run(
            () -> drive.arcadeDrive(forward.getAsDouble(), rotation.getAsDouble()));
    }
}
