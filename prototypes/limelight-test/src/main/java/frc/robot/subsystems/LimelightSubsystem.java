package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
  // Get the Limelight's NetworkTable
  private final NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  
  // Get the 'tv' (target valid) entry
  private final NetworkTableEntry tv = limelightTable.getEntry("tv"); 

  /** Creates a new LimelightSubsystem. */
  public LimelightSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    // It's a good place for pushing data to SmartDashboard, 
    // but the target check will run in Robot.java's periodic() for now.
  }

  /**
   * Checks if the Limelight sees a valid retroreflective target.
   * @return True if a target is visible, false otherwise.
   */
  public boolean hasRetroTarget() {
    // 'tv' is 1.0 when a target is visible, 0.0 otherwise.
    return tv.getDouble(0.0) == 1.0;
  }
}