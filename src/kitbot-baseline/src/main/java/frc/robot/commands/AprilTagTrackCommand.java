package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;

public class AprilTagTrackCommand extends Command {
    private final AprilTagSubsystem aprilTagSubsystem;
    private final DriveSubsystem driveSubsystem;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier rotationSupplier;
    private final double maxSpeed;

    public AprilTagTrackCommand(
        AprilTagSubsystem aprilTagSubsystem,
        DriveSubsystem driveSubsystem,
        DoubleSupplier forwardSupplier,
        DoubleSupplier rotationSupplier,
        double maxSpeed
    ) {
        this.aprilTagSubsystem = aprilTagSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.forwardSupplier = forwardSupplier;
        this.rotationSupplier = rotationSupplier;
        this.maxSpeed = maxSpeed;

        addRequirements(aprilTagSubsystem, driveSubsystem);
    }

    @Override
    public void execute() {
        double forward = forwardSupplier.getAsDouble();
        double rotation = rotationSupplier.getAsDouble();

        // If we have a target, use AprilTag tracking
        if (aprilTagSubsystem.hasTarget()) {
            forward += aprilTagSubsystem.getForwardSpeed();
            rotation += aprilTagSubsystem.getRotationSpeed();
        }

        // Apply speed limits
        forward = Math.max(-maxSpeed, Math.min(maxSpeed, forward));
        rotation = Math.max(-maxSpeed, Math.min(maxSpeed, rotation));

        // Drive the robot
        driveSubsystem.arcadeDrive(forward, rotation);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
