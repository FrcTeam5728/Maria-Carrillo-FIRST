package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.constraints.ConstraintManager;
import frc.robot.subsystems.vision.constraints.DistanceConstraint;
import frc.robot.subsystems.vision.constraints.PositionConstraint;
import frc.robot.subsystems.vision.constraints.RotationConstraint;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class AprilTagSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final ConstraintManager constraintManager;
    private int targetId = -1; // -1 means any visible target

    public AprilTagSubsystem(String cameraName) {
        this.camera = new PhotonCamera(cameraName);
        this.constraintManager = new ConstraintManager();
        
        // Initialize with default constraints
        initializeDefaultConstraints();
        
        // Load the AprilTag field layout
        try {
            aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }
    }

    private void initializeDefaultConstraints() {
        // Add default constraints with reasonable PID values
        constraintManager.addConstraint("distance", new DistanceConstraint(
            "Distance", 1.0, 0.5, 0.0, 0.1));
        
        constraintManager.addConstraint("rotation", new RotationConstraint(
            "Rotation", 0.1, 0.0, 0.01));
        
        constraintManager.addConstraint("position", new PositionConstraint(
            "Position", 0.0, 0.0, 0.5, 0.0, 0.1));
    }

    public void setTargetId(int id) {
        this.targetId = id;
    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return Optional.empty();
        }

        if (targetId == -1) {
            return Optional.ofNullable(result.getBestTarget());
        }
        
        return result.getTargets().stream()
            .filter(target -> target.getFiducialId() == targetId)
            .findFirst();
    }

    public boolean hasTarget() {
        return getBestTarget().isPresent();
    }

    /**
     * Gets the constraint manager for this subsystem.
     * @return The constraint manager
     */
    public ConstraintManager getConstraintManager() {
        return constraintManager;
    }

    /**
     * Calculates the chassis speeds needed to align with the target
     * @return ChassisSpeeds with the required velocities, or null if no target
     */
    public Optional<ChassisSpeeds> getTargetSpeeds() {
        var target = getBestTarget();
        if (target.isEmpty()) return Optional.empty();

        // Get the constraints
        DistanceConstraint distanceConstraint = (DistanceConstraint) constraintManager.getConstraint("distance");
        RotationConstraint rotationConstraint = (RotationConstraint) constraintManager.getConstraint("rotation");
        PositionConstraint positionConstraint = (PositionConstraint) constraintManager.getConstraint("position");

        if (distanceConstraint == null || rotationConstraint == null || positionConstraint == null) {
            DriverStation.reportWarning("One or more required constraints are not set", false);
            return Optional.empty();
        }

        // Calculate speeds based on constraints
        double yaw = target.get().getYaw();
        double distance = target.get().getBestCameraToTarget().getTranslation().getNorm();

        double rotationSpeed = rotationConstraint.calculate(yaw, 0);
        double forwardSpeed = distanceConstraint.calculate(distance, distanceConstraint.getSetpoint());
        
        // TODO: Add position constraint calculations if needed
        
        return Optional.of(new ChassisSpeeds(forwardSpeed, 0, rotationSpeed));
    }

    public Optional<Pose2d> getRobotPose() {
        var target = getBestTarget();
        if (target.isEmpty()) return Optional.empty();
        
        try {
            var targetPose = aprilTagFieldLayout.getTagPose(target.get().getFiducialId());
            if (targetPose.isEmpty()) return Optional.empty();
            
            // TODO: Implement proper pose estimation using the target pose and camera transform
            // This is a placeholder - you'll need to adjust based on your robot's configuration
            return Optional.of(new Pose2d());
        } catch (Exception e) {
            DriverStation.reportError("Failed to get robot pose: " + e.getMessage(), false);
            return Optional.empty();
        }
    }

    @Override
    public void periodic() {
        // Update any periodic tasks here
    }
}
