package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Command to toggle vision-based pose correction on/off.
 */
public class ToggleVisionCorrectionCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    public ToggleVisionCorrectionCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        // No need to require the subsystem since this command doesn't interfere with other commands
    }

    @Override
    public void initialize() {
        boolean current = swerveSubsystem.getPoseEstimator().isVisionCorrectionEnabled();
        swerveSubsystem.getPoseEstimator().setVisionCorrectionEnabled(!current);
    }

    @Override
    public boolean isFinished() {
        return true; // This command runs once and finishes
    }
}
