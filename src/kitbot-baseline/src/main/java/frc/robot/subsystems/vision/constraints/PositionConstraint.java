package frc.robot.subsystems.vision.constraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

/**
 * Constraint to maintain a specific 2D position relative to the AprilTag.
 * This can be used for side-to-side positioning while maintaining distance.
 */
public class PositionConstraint implements AprilTagConstraint {
    private final PIDController xController;
    private final PIDController yController;
    private double targetXOffset;
    private double targetYOffset;
    private boolean isActive;
    private final String name;

    /**
     * Creates a new PositionConstraint
     * @param name Display name for this constraint
     * @param targetXOffset Desired X offset from target (left/right) in meters
     * @param targetYOffset Desired Y offset from target (forward/back) in meters
     * @param kP Proportional gain for position control
     * @param kI Integral gain for position control
     * @param kD Derivative gain for position control
     */
    public PositionConstraint(String name, double targetXOffset, double targetYOffset, 
                            double kP, double kI, double kD) {
        this.name = name;
        this.targetXOffset = targetXOffset;
        this.targetYOffset = targetYOffset;
        this.xController = new PIDController(kP, kI, kD);
        this.yController = new PIDController(kP, kI, kD);
        this.isActive = true;
        
        xController.setTolerance(0.05); // 5cm tolerance
        yController.setTolerance(0.05); // 5cm tolerance
    }

    @Override
    public Optional<ChassisSpeeds> calculate(PhotonTrackedTarget target, Optional<Pose2d> currentPose) {
        if (!isActive) {
            return Optional.empty();
        }

        // Get the 3D transform from camera to target
        var cameraToTarget = target.getBestCameraToTarget();
        
        // Calculate current offsets (convert from 3D to 2D)
        double currentX = cameraToTarget.getX();
        double currentY = cameraToTarget.getY();
        
        // Calculate desired movement in field coordinates
        double xSpeed = -xController.calculate(currentX, targetXOffset);
        double ySpeed = yController.calculate(currentY, targetYOffset);
        
        return Optional.of(new ChassisSpeeds(xSpeed, ySpeed, 0));
    }

    public void setTargetOffsets(double xOffset, double yOffset) {
        this.targetXOffset = xOffset;
        this.targetYOffset = yOffset;
    }

    @Override
    public boolean isActive() {
        return isActive;
    }

    @Override
    public void setActive(boolean active) {
        this.isActive = active;
    }

    @Override
    public String getName() {
        return name;
    }
}
