// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.visualization.RobotVisualization;
import frc.robot.visualization.MechanismVisualization;
import frc.robot.simulation.SimulationManager;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  
  // AdvantageScope NetworkTables publishing
  private DoubleArrayPublisher m_robotPosePub;
  private StringPublisher m_robotModelPub;
  private Pose2d m_robotPose = new Pose2d(0, 0, new Rotation2d());
  
  // Debug counter for model republishing
  private int modelRepublishCounter = 0;
  
  // Mechanism2D visualization for Glass
  private MechanismVisualization m_mechanismViz;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    // Used to track usage of Kitbot code, please do not remove.
    HAL.report(tResourceType.kResourceType_Framework, 10);
    
    // Initialize 3D robot visualization for AdvantageScope
    RobotVisualization.initialize();
    
    // Initialize Mechanism2D visualization for Glass
    m_mechanismViz = new MechanismVisualization();
    
    // Initialize simulation systems (only active in simulation mode)
    SimulationManager.getInstance();
    
    // Publish robot pose to NetworkTables for AdvantageScope
    m_robotPosePub = NetworkTableInstance.getDefault()
        .getDoubleArrayTopic("/AdvantageScope/Robot/Pose").publish();
    
    // Publish robot model description
    m_robotModelPub = NetworkTableInstance.getDefault()
        .getStringTopic("/AdvantageScope/Robot/Model").publish();
    
  // Simple box model for KitBot
  String modelJson = "{\"type\":\"box\",\"length\":0.7,\"width\":0.7,\"height\":0.3}";
  m_robotModelPub.set(modelJson);
  // Robot model published to NetworkTables
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything
    // in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    
    // Update camera server status for Shuffleboard
    updateCameraStatus();
    
    // Update robot visualization
    updateRobotVisualization();
    
    // Update simulation systems
    SimulationManager.getInstance().update();
    
    // Update Mechanism2D visualization
    updateMechanismVisualization();
    
    // Update RobotContainer subsystems
    m_robotContainer.periodic();
    
    // Debug: Check if model publisher is still valid
        if (m_robotModelPub != null) {
      // Occasionally republish model to ensure it's available
      modelRepublishCounter++;
      if (modelRepublishCounter % 250 == 0) { // Every 5 seconds
        String modelJson = "{\"type\":\"box\",\"length\":0.7,\"width\":0.7,\"height\":0.3}";
        m_robotModelPub.set(modelJson);
        // Robot model republished to NetworkTables
      }
    }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  
  /**
   * Update robot position in 3D visualization
   */
  private void updateRobotVisualization() {
    // Get robot pose from drive subsystem if available
    if (m_robotContainer != null) {
      // Use reflection to access drive subsystem since it's private
      try {
        java.lang.reflect.Field driveField = RobotContainer.class.getDeclaredField("driveSubsystem");
        driveField.setAccessible(true);
        Object driveSubsystem = driveField.get(m_robotContainer);
        
        if (driveSubsystem != null) {
          // Try to get pose from drive subsystem
          try {
            java.lang.reflect.Method getPoseMethod = driveSubsystem.getClass().getMethod("getPose");
            Object pose = getPoseMethod.invoke(driveSubsystem);
            
      if (pose instanceof edu.wpi.first.math.geometry.Pose2d) {
        edu.wpi.first.math.geometry.Pose2d newPose = (edu.wpi.first.math.geometry.Pose2d) pose;
        // Only update visualization/print when pose has changed significantly to avoid spamming logs
        boolean shouldUpdate = false;
        if (m_robotPose == null) {
          shouldUpdate = true;
        } else {
          double dx = Math.abs(newPose.getX() - m_robotPose.getX());
          double dy = Math.abs(newPose.getY() - m_robotPose.getY());
          double dheading = Math.abs(newPose.getRotation().getDegrees() - m_robotPose.getRotation().getDegrees());
          if (dx > 0.05 || dy > 0.05 || dheading > 1.0) { // thresholds: 5cm, 1deg
            shouldUpdate = true;
          }
        }

        if (shouldUpdate) {
          m_robotPose = newPose;
          RobotVisualization.updateRobotPosition(m_robotPose);
          // Pose updated for visualization
        }
      }
          } catch (Exception e) {
            // Fallback: send default position if pose can't be retrieved
            RobotVisualization.updateRobotPosition(new edu.wpi.first.math.geometry.Pose2d());
          }
        } else {
          // Drive subsystem is null - using default pose
        }
      } catch (Exception e) {
        // Field access failed, send default position
        RobotVisualization.updateRobotPosition(new edu.wpi.first.math.geometry.Pose2d());
      }
    } else {
      // Drive subsystem is null - no pose update
    }
  }
  
  /**
   * Update Mechanism2D visualization for Glass
   */
  private void updateMechanismVisualization() {
    if (m_mechanismViz != null) {
      // Update robot pose
      m_mechanismViz.updateRobotPose(m_robotPose);
      
      // Get subsystem states for visualization
      SmartDashboard.getBoolean("Limelight/HasTarget", false);
      SmartDashboard.getNumber("Limelight/TX", 0.0);
      SmartDashboard.getNumber("Limelight/TY", 0.0);
      
    } else {
      // MechanismVisualization is null - no update
    }
  }
  
  /**
   * Updates camera server status for Shuffleboard.
   */
  private void updateCameraStatus() {
    // Camera server status handled by CameraFeedBroadcaster
  }
}
