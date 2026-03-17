// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.visualization;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Mechanism2D Visualization for Glass Dashboard
 * Provides 2D robot visualization for WPILib Glass
 */
public class MechanismVisualization {
  
  private Mechanism2d m_mechanism;
  private MechanismLigament2d m_robotBody;
  private MechanismLigament2d m_robotDirection;
  private MechanismLigament2d m_shooterArm;
  private MechanismLigament2d m_intakeIndicator;
  
  // Robot dimensions for visualization
  private static final double ROBOT_LENGTH = 0.7; // meters
  
  public MechanismVisualization() {
    initializeMechanism();
  }
  
  /**
   * Initialize the Mechanism2D visualization
   */
  private void initializeMechanism() {
    // Create mechanism with appropriate dimensions
    m_mechanism = new Mechanism2d(10.0, 10.0);
    
    // Create robot root (center of field)
    var root = m_mechanism.getRoot("Robot", 5.0, 5.0);
    
    // Robot body (main chassis)
    m_robotBody = new MechanismLigament2d(
        "Body", 
        ROBOT_LENGTH, 
        0, 
        6, 
        new Color8Bit(Color.kBlue)
    );
    
    // Direction indicator (shows robot heading)
    m_robotDirection = new MechanismLigament2d(
        "Direction", 
        ROBOT_LENGTH * 0.3, 
        0, 
        4, 
        new Color8Bit(Color.kRed)
    );
    
    // Shooter arm (simulated)
    m_shooterArm = new MechanismLigament2d(
        "Shooter", 
        ROBOT_LENGTH * 0.4, 
        45, 
        3, 
        new Color8Bit(Color.kGreen)
    );
    
    // Intake indicator
    m_intakeIndicator = new MechanismLigament2d(
        "Intake", 
        ROBOT_LENGTH * 0.2, 
        -45, 
        2, 
        new Color8Bit(Color.kOrange)
    );
    
    // Assemble the mechanism
    root.append(m_robotBody);
    m_robotBody.append(m_robotDirection);
    m_robotBody.append(m_shooterArm);
    m_robotBody.append(m_intakeIndicator);
    
    // Publish to SmartDashboard
    SmartDashboard.putData("RobotMechanism", m_mechanism);
    
    System.out.println("Mechanism2D Visualization initialized for Glass");
  }
  
  /**
   * Update robot position and orientation
   */
  public void updateRobotPose(Pose2d robotPose) {
    // Scale field coordinates to mechanism coordinates
    double mechanismX = 5.0 + robotPose.getX();
    double mechanismY = 5.0 + robotPose.getY();
    double mechanismAngle = robotPose.getRotation().getDegrees();
    
    // Update robot position and orientation
    var robotRoot = m_mechanism.getRoot("Robot", mechanismX, mechanismY);
    robotRoot.setPosition(mechanismX, mechanismY);
    m_robotBody.setAngle(mechanismAngle);
    m_robotDirection.setAngle(0); // Always points forward relative to robot
    
    // Update dashboard info
    SmartDashboard.putNumber("Mechanism/Robot/X", robotPose.getX());
    SmartDashboard.putNumber("Mechanism/Robot/Y", robotPose.getY());
    SmartDashboard.putNumber("Mechanism/Robot/Heading", robotPose.getRotation().getDegrees());
  }
  
  /**
   * Update shooter state
   */
  public void updateShooterState(boolean isShooting, double shooterSpeed) {
    if (isShooting) {
      // Extend and change color when shooting
      m_shooterArm.setLength(ROBOT_LENGTH * 0.6);
      m_shooterArm.setColor(new Color8Bit(Color.kYellow));
    } else {
      // Retract when not shooting
      m_shooterArm.setLength(ROBOT_LENGTH * 0.4);
      m_shooterArm.setColor(new Color8Bit(Color.kGreen));
    }
    
    // Update shooter angle based on speed (simulated)
    double shooterAngle = Math.min(90, shooterSpeed * 45); // 0-90 degrees based on speed
    m_shooterArm.setAngle(shooterAngle);
    
    SmartDashboard.putBoolean("Mechanism/Shooter/Active", isShooting);
    SmartDashboard.putNumber("Mechanism/Shooter/Speed", shooterSpeed);
    SmartDashboard.putNumber("Mechanism/Shooter/Angle", shooterAngle);
  }
  
  /**
   * Update intake state
   */
  public void updateIntakeState(boolean isIntaking, boolean hasFuel) {
    if (isIntaking) {
      // Extend and change color when intaking
      m_intakeIndicator.setLength(ROBOT_LENGTH * 0.4);
      m_intakeIndicator.setColor(new Color8Bit(Color.kYellow));
    } else if (hasFuel) {
      // Show presence of fuel
      m_intakeIndicator.setLength(ROBOT_LENGTH * 0.3);
      m_intakeIndicator.setColor(new Color8Bit(Color.kPurple));
    } else {
      // Retract when not intaking
      m_intakeIndicator.setLength(ROBOT_LENGTH * 0.2);
      m_intakeIndicator.setColor(new Color8Bit(Color.kOrange));
    }
    
    SmartDashboard.putBoolean("Mechanism/Intake/Active", isIntaking);
    SmartDashboard.putBoolean("Mechanism/Intake/HasFuel", hasFuel);
  }
  
  /**
   * Update Limelight targeting state
   */
  public void updateLimelightState(boolean hasTarget, double tx, double ty) {
    if (hasTarget) {
      // Change robot body color when target is acquired
      m_robotBody.setColor(new Color8Bit(Color.kLimeGreen));
      
      // Adjust direction indicator to show target offset
      m_robotDirection.setAngle(tx); // Use horizontal offset as angle
      m_robotDirection.setLength(ROBOT_LENGTH * 0.5); // Extend when targeting
    } else {
      // Normal color when no target
      m_robotBody.setColor(new Color8Bit(Color.kBlue));
      
      // Reset direction indicator
      m_robotDirection.setAngle(0);
      m_robotDirection.setLength(ROBOT_LENGTH * 0.3);
    }
    
    SmartDashboard.putBoolean("Mechanism/Limelight/HasTarget", hasTarget);
    SmartDashboard.putNumber("Mechanism/Limelight/TX", tx);
    SmartDashboard.putNumber("Mechanism/Limelight/TY", ty);
  }
  
  /**
   * Update autonomous state
   */
  public void updateAutonomousState(String state, String action) {
    // Change robot color based on autonomous state
    switch (state.toUpperCase()) {
      case "READY":
        m_robotBody.setColor(new Color8Bit(Color.kBlue));
        break;
      case "NAVIGATING":
        m_robotBody.setColor(new Color8Bit(Color.kCyan));
        break;
      case "SCORING":
        m_robotBody.setColor(new Color8Bit(Color.kGreen));
        break;
      case "COLLECTING":
        m_robotBody.setColor(new Color8Bit(Color.kOrange));
        break;
      case "AVOIDING":
        m_robotBody.setColor(new Color8Bit(Color.kRed));
        break;
      default:
        m_robotBody.setColor(new Color8Bit(Color.kBlue));
        break;
    }
    
    SmartDashboard.putString("Mechanism/Autonomous/State", state);
    SmartDashboard.putString("Mechanism/Autonomous/Action", action);
  }
  
  /**
   * Reset mechanism to default state
   */
  public void reset() {
    m_robotBody.setColor(new Color8Bit(Color.kBlue));
    m_robotBody.setAngle(0);
    m_robotDirection.setAngle(0);
    m_robotDirection.setLength(ROBOT_LENGTH * 0.3);
    m_shooterArm.setAngle(45);
    m_shooterArm.setLength(ROBOT_LENGTH * 0.4);
    m_shooterArm.setColor(new Color8Bit(Color.kGreen));
    m_intakeIndicator.setAngle(-45);
    m_intakeIndicator.setLength(ROBOT_LENGTH * 0.2);
    m_intakeIndicator.setColor(new Color8Bit(Color.kOrange));
    
    System.out.println("Mechanism2D Visualization reset to default state");
  }
}
