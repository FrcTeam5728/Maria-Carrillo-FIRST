package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.pose_estimation.OdometryPoseEstimator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * Command that allows setting a waypoint with one trigger and returning to it with another.
 * Demonstrates the use of odometry-based position control.
 */
public class WaypointCommand extends Command {
    private final SwerveSubsystem swerve;
    private final OdometryPoseEstimator poseEstimator;
    private final Trigger setWaypointTrigger;
    private final Trigger returnToWaypointTrigger;
    
    private Pose2d waypoint = null;
    private boolean isReturning = false;

    /**
     * Creates a new WaypointCommand.
     *
     * @param swerve The swerve subsystem to control
     * @param poseEstimator The pose estimator to get position data from
     * @param setWaypointTrigger Trigger to set the current position as a waypoint
     * @param returnToWaypointTrigger Trigger to start returning to the waypoint
     */
    public WaypointCommand(
        SwerveSubsystem swerve,
        OdometryPoseEstimator poseEstimator,
        Trigger setWaypointTrigger,
        Trigger returnToWaypointTrigger
    ) {
        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.setWaypointTrigger = setWaypointTrigger;
        this.returnToWaypointTrigger = returnToWaypointTrigger;
        
        // This command doesn't require the swerve subsystem exclusively
        // so it can run alongside other commands
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Reset any existing waypoint behavior
        isReturning = false;
        System.out.println("Waypoint Command Initialized");
    }

    @Override
    public void execute() {
        // Set waypoint when the set trigger is pressed
        if (setWaypointTrigger.getAsBoolean()) {
            waypoint = poseEstimator.getEstimatedPose();
            System.out.println("Waypoint set at: (" + 
                String.format("%.2f", waypoint.getX()) + ", " + 
                String.format("%.2f", waypoint.getY()) + ")");
            isReturning = false;
        }
        
        // Return to waypoint when the return trigger is pressed and we have a waypoint
        if (returnToWaypointTrigger.getAsBoolean() && waypoint != null) {
            isReturning = true;
        }
        
        // If we're returning to the waypoint, drive there
        if (isReturning && waypoint != null) {
            // Get current position and calculate errors
            Pose2d currentPose = poseEstimator.getEstimatedPose();
            double xError = waypoint.getX() - currentPose.getX();
            double yError = waypoint.getY() - currentPose.getY();
            double distance = Math.hypot(xError, yError);
            
            // Stop if we're close enough (10cm tolerance)
            if (distance < 0.1) {
                swerve.drive(0, 0, 0, false);
                isReturning = false;
                System.out.println("Reached waypoint!");
                return;
            }
            
            // Calculate field-relative speeds using a P controller
            double maxSpeed = 1.0;  // m/s
            double pGain = 2.0;     // Proportional gain
            
            // Calculate speeds with limits
            double xSpeed = Math.max(-maxSpeed, Math.min(xError * pGain, maxSpeed));
            double ySpeed = Math.max(-maxSpeed, Math.min(yError * pGain, maxSpeed));
            
            // Drive towards the waypoint (field-relative)
            swerve.drive(xSpeed, ySpeed, 0, true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        swerve.drive(0, 0, 0, false);
        System.out.println("Waypoint Command Ended" + (interrupted ? " (Interrupted)" : ""));
    }

    @Override
    public boolean isFinished() {
        // This command runs continuously until interrupted
        return false;
    }
}
