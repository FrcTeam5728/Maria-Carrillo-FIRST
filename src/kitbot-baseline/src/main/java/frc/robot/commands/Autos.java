// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathPlannerPath;

public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(.25),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        driveSubsystem.driveArcade(() -> 0, () -> 0),
        // Repeat pattern: shoot for 2 seconds, intake for 1 second, for 15 seconds total
        // This creates 5 complete cycles (5 * 3 = 15 seconds)
        new SequentialCommandGroup(
            ballSubsystem.launchCommand().withTimeout(2),
            ballSubsystem.runOnce(() -> ballSubsystem.intake()).withTimeout(1)
        ).repeatedly().withTimeout(15),
        // Stop running the launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop()));
  }

  /*
   * PathPlanner autonomous that follows "example" path.
   * 
   * @param driveSubsystem The differential drive subsystem
   * @param ballSubsystem The fuel subsystem
   * @return Command that follows path and shoots
   */
  /*
  public static final Command pathPlannerExampleAuto(DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Follow example path
        FollowPathPlannerPath.followNamedPath("example", driveSubsystem, true),
        
        // Stop and shoot for 3 seconds
        driveSubsystem.driveArcade(() -> 0, () -> 0).alongWith(
            ballSubsystem.launchCommand().withTimeout(3)
        ),
        
        // Stop launcher
        ballSubsystem.runOnce(() -> ballSubsystem.stop())
    );
  }

  /*
   * Creates a PathPlanner autonomous command from a named auto.
   * 
   * @param autoName The name of the auto from autos.json
   * @param driveSubsystem The differential drive subsystem  
   * @param ballSubsystem The fuel subsystem
   * @return Command that runs the full PathPlanner autonomous
   */
  /*
  public static final Command pathPlannerAuto(String autoName, DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Run the PathPlanner auto
        FollowPathPlannerPath.createAutoCommand(autoName),
        
        // Ensure everything is stopped at the end
        driveSubsystem.driveArcade(() -> 0, () -> 0).alongWith(
            ballSubsystem.runOnce(() -> ballSubsystem.stop())
        )
    );
  }

  /*
   * Custom PathPlanner auto with path following and shooting sequence.
   * 
   * @param driveSubsystem The differential drive subsystem
   * @param ballSubsystem The fuel subsystem
   * @return Command with path following and shooting
   */
  /*
  public static final Command customPathPlannerAuto(DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
    return new SequentialCommandGroup(
        // Follow example path
        FollowPathPlannerPath.followNamedPath("example", driveSubsystem, true),
        
        // Repeat pattern: shoot for 2 seconds, intake for 1 second, for 15 seconds total
        new SequentialCommandGroup(
            ballSubsystem.launchCommand().withTimeout(2),
            ballSubsystem.intakeCommand().withTimeout(1)
        ).repeatedly().withTimeout(15),
        
        // Stop everything
        driveSubsystem.driveArcade(() -> 0, () -> 0).alongWith(
            ballSubsystem.runOnce(() -> ballSubsystem.stop())
        )
    );
  }
  */
}
