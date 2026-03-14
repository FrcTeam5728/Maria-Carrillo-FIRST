// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureIO;
import frc.robot.subsystems.superstructure.SuperstructureIOSim;
import frc.robot.subsystems.superstructure.SuperstructureIOSpark;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Superstructure superstructure;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // drive = new Drive(new DriveIOTalonSRX(), new GyroIOPigeon2());
        drive = new Drive(new DriveIOSpark(), new GyroIONavX());
        superstructure = new Superstructure(new SuperstructureIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        superstructure = new Superstructure(new SuperstructureIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        superstructure = new Superstructure(new SuperstructureIO() {});
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand("Launch", superstructure.launch().withTimeout(6.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default drive command, normal arcade drive
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> -controller.getLeftY(), () -> -controller.getRightX()));

    // Control bindings for superstructure
    controller.leftBumper().whileTrue(superstructure.intake());
    controller.rightBumper().whileTrue(superstructure.launch());
    controller.a().whileTrue(superstructure.eject());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}

// Below is from the AdvantageKit vision template. The subsystem files are already included,
// but it is not currently used by the main RobotContainer. If applying their vision subsystem is
// desired,
// pick out code from the commented out sections below.

// // Copyright (c) 2021-2026 Littleton Robotics
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by a BSD
// // license that can be found in the LICENSE file
// // at the root directory of this project.

// package frc.robot;

// import static frc.robot.subsystems.vision.VisionConstants.*;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.GenericHID;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
// import frc.robot.subsystems.drive.DemoDrive;
// import frc.robot.subsystems.vision.Vision;
// import frc.robot.subsystems.vision.VisionIO;
// import frc.robot.subsystems.vision.VisionIOLimelight;
// import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in the {@link
// Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of the robot
// (including
//  * subsystems, commands, and button mappings) should be declared here.
//  */
// public class RobotContainer {
//   private final Vision vision;

//   private final DemoDrive drive = new DemoDrive(); // Demo drive subsystem, sim only
//   private final CommandGenericHID keyboard = new CommandGenericHID(0); // Keyboard 0 on port 0

//   /** The container for the robot. Contains subsystems, OI devices, and commands. */
//   public RobotContainer() {
//     switch (Constants.currentMode) {
//       case REAL:
//         // Real robot, instantiate hardware IO implementations
//         vision =
//             new Vision(
//                 drive::addVisionMeasurement,
//                 new VisionIOLimelight(camera0Name, drive::getRotation),
//                 new VisionIOLimelight(camera1Name, drive::getRotation));
//         // vision =
//         // new Vision(
//         // demoDrive::addVisionMeasurement,
//         // new VisionIOPhotonVision(camera0Name, robotToCamera0),
//         // new VisionIOPhotonVision(camera1Name, robotToCamera1));
//         break;

//       case SIM:
//         // Sim robot, instantiate physics sim IO implementations
//         vision =
//             new Vision(
//                 drive::addVisionMeasurement,
//                 new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
//                 new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
//         break;

//       default:
//         // Replayed robot, disable IO implementations
//         // (Use same number of dummy implementations as the real robot)
//         vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
//         break;
//     }

//     // Configure the button bindings
//     configureButtonBindings();
//   }

//   /**
//    * Use this method to define your button->command mappings. Buttons can be created by
//    * instantiating a {@link GenericHID} or one of its subclasses ({@link
//    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
//    * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
//    */
//   private void configureButtonBindings() {
//     // Joystick drive command
//     drive.setDefaultCommand(
//         Commands.run(
//             () -> {
//               drive.run(-keyboard.getRawAxis(1), -keyboard.getRawAxis(0));
//             },
//             drive));

//     // Auto aim command example
//     @SuppressWarnings("resource")
//     PIDController aimController = new PIDController(0.2, 0.0, 0.0);
//     aimController.enableContinuousInput(-Math.PI, Math.PI);
//     keyboard
//         .button(1)
//         .whileTrue(
//             Commands.startRun(
//                 () -> {
//                   aimController.reset();
//                 },
//                 () -> {
//                   drive.run(0.0, aimController.calculate(vision.getTargetX(0).getRadians()));
//                 },
//                 drive));
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     return Commands.none();
//   }
// }
