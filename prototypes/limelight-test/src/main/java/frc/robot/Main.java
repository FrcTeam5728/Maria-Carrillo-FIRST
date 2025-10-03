package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Do NOT add any static variables to this class, or any initialization at all. Unless you know what
 * you are doing, do not modify this file except to change the parameter type on the call to the
 * {@link RobotBase#startRobot} method.
 */
public final class Main {
  private Main() {
    /* private constructor to prevent instantiation */
  }

  /**
   * Main method.
   *
   * @param args The command line arguments.
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}