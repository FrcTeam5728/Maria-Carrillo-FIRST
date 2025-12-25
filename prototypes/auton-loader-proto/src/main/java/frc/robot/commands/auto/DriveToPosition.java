package frc.robot.commands.auto;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveController;

public class DriveToPosition extends Command {
    private final SwerveSubsystem drivebase;
    private final double targetX;
    private final double targetY;
    private final Rotation2d targetRotation;
    
    // Tolerances (in meters and degrees)
    private final double positionTolerance = 0.05; // 5cm
    private final double rotationTolerance = 2.0;  // 2 degrees
    
    // PID Controllers
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;
    
    // Max speeds
    private final double maxSpeedMetersPerSecond = 3.0; // Adjust based on your robot's capabilities
    private final double maxAngularSpeedRadPerSecond = Math.PI; // 180 degrees per second

    public DriveToPosition(SwerveSubsystem drivebase, double x, double y, Rotation2d rotation) {
        this.drivebase = drivebase;
        this.targetX = x;
        this.targetY = y;
        this.targetRotation = rotation;
        
        // Initialize PID controllers with gains (adjust these values based on your robot)
        xController = new ProfiledPIDController(2.0, 0, 0.1, 
            new TrapezoidProfile.Constraints(maxSpeedMetersPerSecond, 3.0));
        
        yController = new ProfiledPIDController(2.0, 0, 0.1, 
            new TrapezoidProfile.Constraints(maxSpeedMetersPerSecond, 3.0));
            
        thetaController = new ProfiledPIDController(3.0, 0, 0.1, 
            new TrapezoidProfile.Constraints(maxAngularSpeedRadPerSecond, Math.PI));
        
        // Make thetaController continuous (it's an angle controller)
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        // Set tolerances
        xController.setTolerance(positionTolerance);
        yController.setTolerance(positionTolerance);
        thetaController.setTolerance(Math.toRadians(rotationTolerance));
        
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        // Reset PID controllers with current position
        Pose2d currentPose = drivebase.getPose();
        
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());
        
        // Set setpoints (target positions/orientation)
        xController.setGoal(targetX);
        yController.setGoal(targetY);
        thetaController.setGoal(targetRotation.getRadians());
    }

    @Override
    public void execute() {
        // Get current pose
        Pose2d currentPose = drivebase.getPose();
        
        // Calculate speeds using PID controllers
        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());
        double rotSpeed = thetaController.calculate(currentPose.getRotation().getRadians());
        
        // Create chassis speeds
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xSpeed, ySpeed, rotSpeed, currentPose.getRotation()
        );
        
        // Apply the speeds to the drivebase
        drivebase.driveFieldOriented(speeds);
    }

    @Override
    public boolean isFinished() {
        // Check if we've reached the target position and rotation
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drivetrain when the command ends
        drivebase.driveFieldOriented(new ChassisSpeeds(0, 0, 0));
    }
}
