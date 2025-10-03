package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually reside in the {@link Robot}
 * periodic methods, but rather in the {@link Command} and {@link Subsystem} classes.
 */
public class RobotContainer {
  
  // This instantiates the Limelight Subsystem once when the robot starts.
  public static final LimelightSubsystem m_limelight = new LimelightSubsystem();

  private final XboxController m_driverController = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button to command mappings.
   */
  private void configureButtonBindings() {
    // Example: Bind a button to a command here
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example autonomous command
    return null; // Return a real command here when you implement autonomous
  }
}