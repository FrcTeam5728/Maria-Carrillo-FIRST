package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingDistanceControl;

/**
 * Command that provides continuous distance control using D-pad.
 * Allows holding D-pad buttons for continuous adjustment.
 */
public class DistanceControlCommand extends CommandBase {
  
  private final ShootingDistanceControl distanceControl;
  private final boolean increasing;
  private static final double ADJUSTMENT_RATE = 0.5; // feet per second while holding
  
  /**
   * Creates a new DistanceControlCommand.
   * 
   * @param distanceControl The distance control subsystem
   * @param increasing True to increase distance, false to decrease
   */
  public DistanceControlCommand(ShootingDistanceControl distanceControl, boolean increasing) {
    this.distanceControl = distanceControl;
    this.increasing = increasing;
    
    addRequirements(distanceControl);
  }
  
  @Override
  public void execute() {
    if (increasing) {
      distanceControl.increaseDistance();
    } else {
      distanceControl.decreaseDistance();
    }
  }
  
  @Override
  public boolean isFinished() {
    // Command finishes when at the limit
    if (increasing) {
      return distanceControl.isAtMaxDistance();
    } else {
      return distanceControl.isAtMinDistance();
    }
  }
}
