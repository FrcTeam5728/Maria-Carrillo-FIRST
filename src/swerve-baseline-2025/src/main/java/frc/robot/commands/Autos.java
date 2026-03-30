// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.swervedrive.Limelight;
import java.util.HashMap;
import java.util.Map;

/**
 * Factory class for creating autonomous commands using PathPlanner.
 * Provides methods to load and execute autonomous paths.
 */
public final class Autos {
  
  private static boolean configured = false;
  
  /**
   * Configures PathPlanner AutoBuilder with the swerve drive subsystem.
   * This should be called once during robot initialization.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @param limelight The limelight subsystem (can be null if not using vision)
   */
  public static void configurePathPlanner(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    if (configured) return;
    
    // Configure AutoBuilder for swerve drive
    AutoBuilder.configureHolonomic(
        swerveSubsystem::getPose, // Pose supplier
        swerveSubsystem::resetOdometry, // Pose resetter
        swerveSubsystem::getChassisSpeeds, // Chassis speeds supplier
        swerveSubsystem::drive, // Chassis speeds consumer
        swerveSubsystem.getSwerveDriveController(), // Path following controller
        swerveSubsystem::getCurrentTrajectory, // Path flipper
        swerveSubsystem // Subsystem requirements
    );
    
    // Register named commands for use in PathPlanner
    registerNamedCommands(swerveSubsystem, limelight);
    
    configured = true;
  }
  
  /**
   * Registers named commands that can be used in PathPlanner paths.
   */
  private static void registerNamedCommands(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Example named commands - customize based on your robot's mechanisms
    
    // Balance command for charging station
    NamedCommands.registerCommand("balance", Commands.runOnce(() -> {
      // Add balancing logic here
    }, swerveSubsystem));
    
    // Intake command
    NamedCommands.registerCommand("intake", Commands.runOnce(() -> {
      // Add intake logic here
    }));
    
    // Shoot command
    NamedCommands.registerCommand("shoot", Commands.runOnce(() -> {
      // Add shooting logic here
    }));
    
    // Vision alignment command if limelight is available
    if (limelight != null) {
      NamedCommands.registerCommand("visionAlign", Commands.runOnce(() -> {
        // Add vision alignment logic using limelight
        if (limelight.hasTarget()) {
          // Align to target using limelight data
          double tx = limelight.getTx();
          // Add alignment logic here
        }
      }, swerveSubsystem));
    }
    
    // Wait command
    NamedCommands.registerCommand("wait", Commands.waitSeconds(1.0));
  }
  
  /**
   * Creates an autonomous command from a PathPlanner path name.
   * 
   * @param pathName The name of the PathPlanner path (without .path extension)
   * @return Command that follows the path
   */
  public static Command getPathPlannerAuto(String pathName) {
    return new PathPlannerAuto(pathName);
  }
  
  /**
   * Creates a simple autonomous command that drives forward.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @param distanceMeters Distance to drive forward in meters
   * @param speedMps Speed to drive at in meters per second
   * @return Command that drives forward
   */
  public static Command driveForward(SwerveSubsystem swerveSubsystem, double distanceMeters, double speedMps) {
    return Commands.sequence(
        Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
        Commands.run(() -> {
          swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(speedMps, 0, 0));
        }, swerveSubsystem).withTimeout(distanceMeters / speedMps),
        Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0)))
    );
  }
  
  /**
   * Creates a vision-based autonomous command using Limelight.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @param limelight The limelight subsystem
   * @return Vision-based autonomous command
   */
  public static Command visionAuto(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    return Commands.sequence(
        Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
        Commands.waitUntil(() -> limelight.hasTarget()),
        Commands.run(() -> {
          if (limelight.hasTarget()) {
            double tx = limelight.getTx();
            double ty = limelight.getTy();
            double distance = limelight.getDistanceToTarget();
            
            // Simple proportional control for alignment
            double xSpeed = 0.5;
            double ySpeed = 0.0;
            double rotationSpeed = -tx * 0.02; // Proportional control for rotation
            
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(xSpeed, ySpeed, rotationSpeed));
          }
        }, swerveSubsystem).withTimeout(5.0),
        Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0)))
    );
  }
  
  /**
   * Creates a simple 2-meter forward autonomous command using PathPlanner.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @return Command that drives forward 2 meters
   */
  public static Command simpleForwardAuto(SwerveSubsystem swerveSubsystem) {
    // Try to load a PathPlanner path, fallback to hardcoded movement if not found
    try {
      return getPathPlannerAuto("SimpleForward");
    } catch (Exception e) {
      System.out.println("PathPlanner path 'SimpleForward' not found, using hardcoded movement");
      return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 0, 0));
          }, swerveSubsystem).withTimeout(2.0),
          Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0)))
      );
    }
  }
  
  /**
   * Creates a square pattern autonomous command using PathPlanner.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @return Command that drives in a square pattern
   */
  public static Command squarePatternAuto(SwerveSubsystem swerveSubsystem) {
    // Try to load a PathPlanner path, fallback to hardcoded movement if not found
    try {
      return getPathPlannerAuto("SquarePattern");
    } catch (Exception e) {
      System.out.println("PathPlanner path 'SquarePattern' not found, using hardcoded movement");
      return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
          // Drive forward
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 0, 0));
          }, swerveSubsystem).withTimeout(2.0),
          // Strafe right
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 1.0, 0));
          }, swerveSubsystem).withTimeout(2.0),
          // Drive backward
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(-1.0, 0, 0));
          }, swerveSubsystem).withTimeout(2.0),
          // Strafe left
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, -1.0, 0));
          }, swerveSubsystem).withTimeout(2.0),
          Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0)))
      );
    }
  }
  
  /**
   * Creates a test autonomous command using PathPlanner paths.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @return Test autonomous command
   */
  public static Command testMovementAuto(SwerveSubsystem swerveSubsystem) {
    // Try to load a PathPlanner path, fallback to hardcoded movement if not found
    try {
      return getPathPlannerAuto("TestMovement");
    } catch (Exception e) {
      System.out.println("PathPlanner path 'TestMovement' not found, using hardcoded movement");
      return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
          Commands.print("Starting test movement autonomous"),
          
          // Forward movement
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0.5, 0, 0));
          }, swerveSubsystem).withTimeout(1.5),
          Commands.print("Completed forward movement"),
          
          // Rotation in place
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, Math.PI/4));
          }, swerveSubsystem).withTimeout(2.0),
          Commands.print("Completed rotation"),
          
          // Strafe movement
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0.5, 0));
          }, swerveSubsystem).withTimeout(1.5),
          Commands.print("Completed strafe movement"),
          
          // Diagonal movement
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0.5, 0.5, 0));
          }, swerveSubsystem).withTimeout(1.5),
          Commands.print("Completed diagonal movement"),
          
          Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))),
          Commands.print("Test movement autonomous completed")
      );
    }
  }
  
  /**
   * Creates a score and backup autonomous command using PathPlanner.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @param limelight The limelight subsystem for targeting
   * @return Command that scores and backs up
   */
  public static Command scoreAndBackupAuto(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Try to load a PathPlanner path, fallback to hardcoded movement if not found
    try {
      return getPathPlannerAuto("ScoreAndBackup");
    } catch (Exception e) {
      System.out.println("PathPlanner path 'ScoreAndBackup' not found, using hardcoded movement");
      return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
          Commands.print("Starting score and backup autonomous"),
          
          // Drive forward to scoring position
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 0, 0));
          }, swerveSubsystem).withTimeout(2.0),
          
          // Score (placeholder for actual scoring mechanism)
          Commands.print("Scoring..."),
          Commands.waitSeconds(1.0),
          Commands.print("Score complete"),
          
          // Backup
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(-1.0, 0, 0));
          }, swerveSubsystem).withTimeout(2.0),
          
          Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))),
          Commands.print("Score and backup autonomous completed")
      );
    }
  }
  
  /**
   * Creates a vision alignment and drive autonomous command using PathPlanner.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @param limelight The limelight subsystem
   * @return Command that aligns to target and drives
   */
  public static Command visionAlignAndDriveAuto(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Try to load a PathPlanner path, fallback to hardcoded movement if not found
    try {
      return getPathPlannerAuto("VisionAlignAndDrive");
    } catch (Exception e) {
      System.out.println("PathPlanner path 'VisionAlignAndDrive' not found, using hardcoded movement");
      return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
          Commands.print("Starting vision alignment and drive autonomous"),
          
          // Wait for target
          Commands.waitUntil(() -> limelight.hasTarget()).withTimeout(3.0),
          Commands.print("Target acquired"),
          
          // Align to target
          Commands.run(() -> {
            if (limelight.hasTarget()) {
              double tx = limelight.getTx();
              double rotationSpeed = -tx * 0.02; // Proportional control
              
              // Only rotate, don't move forward yet
              swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, rotationSpeed));
            }
          }, swerveSubsystem).withTimeout(2.0),
          
          // Drive forward while maintaining alignment
          Commands.run(() -> {
            if (limelight.hasTarget()) {
              double tx = limelight.getTx();
              double rotationSpeed = -tx * 0.02;
              
              // Drive forward while maintaining alignment
              swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0.5, 0, rotationSpeed));
            } else {
              // If no target, just drive forward
              swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0.5, 0, 0));
            }
          }, swerveSubsystem).withTimeout(3.0),
          
          Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))),
          Commands.print("Vision alignment and drive autonomous completed")
      );
    }
  }
  
  /**
   * Creates a complex autonomous using PathPlanner paths.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @param limelight The limelight subsystem
   * @return Complex autonomous command
   */
  public static Command complexAuto(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Try to load a PathPlanner path, fallback to hardcoded movement if not found
    try {
      return getPathPlannerAuto("ComplexAuto");
    } catch (Exception e) {
      System.out.println("PathPlanner path 'ComplexAuto' not found, using hardcoded movement");
      return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
          Commands.print("Starting complex autonomous"),
          
          // Phase 1: Drive forward and strafe
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 0.5, 0));
          }, swerveSubsystem).withTimeout(2.0),
          
          // Phase 2: Rotate
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, Math.PI/4));
          }, swerveSubsystem).withTimeout(1.5),
          
          // Phase 3: Vision alignment if target available
          Commands.either(
              Commands.sequence(
                  Commands.print("Aligning to target"),
                  Commands.run(() -> {
                    if (limelight.hasTarget()) {
                      double tx = limelight.getTx();
                      double rotationSpeed = -tx * 0.02;
                      swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, rotationSpeed));
                    }
                  }, swerveSubsystem).withTimeout(2.0)
              ),
              Commands.sequence(
                  Commands.print("No target found, continuing without alignment"),
                  Commands.waitSeconds(2.0)
              ),
              () -> limelight.hasTarget()
          ),
          
          // Phase 4: Final approach
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0.8, 0, 0));
          }, swerveSubsystem).withTimeout(1.5),
          
          Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))),
          Commands.print("Complex autonomous completed")
      );
    }
  }
  
  /**
   * Creates a game piece pickup and score autonomous using PathPlanner.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @param limelight The limelight subsystem
   * @return Command that picks up game piece and scores
   */
  public static Command pickupAndScoreAuto(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Try to load a PathPlanner path, fallback to hardcoded movement if not found
    try {
      return getPathPlannerAuto("PickupAndScore");
    } catch (Exception e) {
      System.out.println("PathPlanner path 'PickupAndScore' not found, using hardcoded movement");
      return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
          Commands.print("Starting pickup and score autonomous"),
          
          // Drive to game piece
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(1.5, 0, 0));
          }, swerveSubsystem).withTimeout(1.5),
          
          // Pickup game piece
          Commands.print("Picking up game piece"),
          Commands.waitSeconds(1.0),
          
          // Turn towards scoring position
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, Math.PI/2));
          }, swerveSubsystem).withTimeout(1.0),
          
          // Drive to scoring position
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 0, 0));
          }, swerveSubsystem).withTimeout(2.0),
          
          // Score game piece
          Commands.print("Scoring game piece"),
          Commands.waitSeconds(1.0),
          
          Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))),
          Commands.print("Pickup and score autonomous completed")
      );
    }
  }
  
  /**
   * Creates a three-note autonomous routine using PathPlanner.
   * 
   * @param swerveSubsystem The swerve drive subsystem
   * @param limelight The limelight subsystem
   * @return Command that scores three game pieces
   */
  public static Command threeNoteAuto(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    // Try to load a PathPlanner path, fallback to hardcoded movement if not found
    try {
      return getPathPlannerAuto("ThreeNote");
    } catch (Exception e) {
      System.out.println("PathPlanner path 'ThreeNote' not found, using hardcoded movement");
      return Commands.sequence(
          Commands.runOnce(() -> swerveSubsystem.resetOdometry()),
          Commands.print("Starting three note autonomous"),
          
          // First note
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(1.0, 0, 0));
          }, swerveSubsystem).withTimeout(1.0),
          Commands.print("Scoring first note"),
          Commands.waitSeconds(0.5),
          
          // Move to second note
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 1.0, 0));
          }, swerveSubsystem).withTimeout(1.5),
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(-1.0, 0, 0));
          }, swerveSubsystem).withTimeout(1.0),
          Commands.print("Scoring second note"),
          Commands.waitSeconds(0.5),
          
          // Move to third note
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, -1.0, 0));
          }, swerveSubsystem).withTimeout(1.5),
          Commands.run(() -> {
            swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(-1.0, 0, 0));
          }, swerveSubsystem).withTimeout(1.0),
          Commands.print("Scoring third note"),
          Commands.waitSeconds(0.5),
          
          Commands.runOnce(() -> swerveSubsystem.drive(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0))),
          Commands.print("Three note autonomous completed")
      );
    }
  }
  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
