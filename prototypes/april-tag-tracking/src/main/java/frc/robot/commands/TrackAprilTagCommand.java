package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class TrackAprilTagCommand extends Command {
    private final LimelightSubsystem limelight;
    private final SwerveSubsystem swerve;
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;
    
    // PID Constants - these may need tuning
    private static final double kP = 0.05;
    private static final double kI = 0.0;
    private static final double kD = 0.01;
    
    // Distance to maintain from the target (in meters)
    private static final double TARGET_DISTANCE = 1.0;
    
    // Maximum speeds
    private static final double MAX_LINEAR_SPEED = 1.0; // m/s
    private static final double MAX_ANGULAR_SPEED = Math.PI; // rad/s
    
    public TrackAprilTagCommand(LimelightSubsystem limelight, SwerveSubsystem swerve) {
        this.limelight = limelight;
        this.swerve = swerve;
        
        // Initialize PID controllers
        xController = new PIDController(kP, kI, kD);
        yController = new PIDController(kP, kI, kD);
        rotationController = new PIDController(0.1, 0, 0.01); // Tighter control for rotation
        
        // Set tolerance for PID controllers (in meters and radians)
        xController.setTolerance(0.05);
        yController.setTolerance(0.05);
        rotationController.setTolerance(Math.toRadians(2));
        
        // Set setpoints (we want to drive to 0,0,0 relative to the target)
        xController.setSetpoint(0);
        yController.setSetpoint(0);
        rotationController.setSetpoint(0);
        
        addRequirements(limelight, swerve);
    }
    
    @Override
    public void initialize() {
        // Turn on the Limelight LED when command starts
        limelight.setLedMode(LimelightSubsystem.LedMode.ON);
        
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
    }
    
    @Override
    public void execute() {
        if (limelight.hasTarget()) {
            // Get target information from Limelight
            double tx = limelight.getTx();
            double ty = limelight.getTy();
            double ta = limelight.getTa();
            
            // Calculate distance to target (this is a simplified calculation)
            // You may need to adjust these values based on your camera setup and target height
            double distance = (1.0 / ta) * 2.0; // Simple approximation
            
            // Calculate desired velocities using PID controllers
            double xSpeed = -xController.calculate(tx / 27.0); // Normalize to -1 to 1
            double ySpeed = -yController.calculate(ty / 20.5); // Normalize to -1 to 1
            double rotationSpeed = -rotationController.calculate(Math.toRadians(tx));
            
            // Scale speeds to maximum values
            xSpeed = Math.copySign(Math.min(Math.abs(xSpeed), MAX_LINEAR_SPEED), xSpeed);
            ySpeed = Math.copySign(Math.min(Math.abs(ySpeed), MAX_LINEAR_SPEED), ySpeed);
            rotationSpeed = Math.copySign(Math.min(Math.abs(rotationSpeed), MAX_ANGULAR_SPEED), rotationSpeed);
            
            // Drive the robot
            swerve.drive(
                new edu.wpi.first.math.geometry.Translation2d(xSpeed, ySpeed),
                rotationSpeed,
                true // Field-relative
            );
            
            // Print debug information
            SmartDashboard.putNumber("Limelight/TX", tx);
            SmartDashboard.putNumber("Limelight/TY", ty);
            SmartDashboard.putNumber("Limelight/TA", ta);
            SmartDashboard.putNumber("Limelight/Distance", distance);
            SmartDashboard.putNumber("TrackAprilTag/X Speed", xSpeed);
            SmartDashboard.putNumber("TrackAprilTag/Y Speed", ySpeed);
            SmartDashboard.putNumber("TrackAprilTag/Rotation", rotationSpeed);
        } else {
            // No target found, stop the robot
            swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, true);
            
            // Print debug information
            SmartDashboard.putString("Limelight/Status", "No target found");
        }
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        swerve.drive(new edu.wpi.first.math.geometry.Translation2d(0, 0), 0, true);
        
        // Turn off the Limelight LED
        limelight.setLedMode(LimelightSubsystem.LedMode.OFF);
    }
    
    @Override
    public boolean isFinished() {
        // End when the robot is within tolerance of the target
        // You can adjust these tolerances as needed
        return xController.atSetpoint() && 
               yController.atSetpoint() && 
               rotationController.atSetpoint() &&
               limelight.hasTarget();
    }
}
