package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Small helper to publish diagnostic information to Shuffleboard/SmartDashboard
 * instead of printing to console. Keeps keys under a `Debug/` prefix.
 */
public final class DashboardLogger {
  private static final String PREFIX = "Debug/";

  private DashboardLogger() {}

  public static void putString(String key, String value) {
    try {
      SmartDashboard.putString(PREFIX + key, value);
    } catch (Throwable t) {
      // best-effort, avoid throwing from diagnostics
      try {
        NetworkTableInstance.getDefault().getEntry(PREFIX + key).setString(value);
      } catch (Throwable ignore) {}
    }
  }

  public static void putNumber(String key, double value) {
    try {
      SmartDashboard.putNumber(PREFIX + key, value);
    } catch (Throwable t) {
      try {
        NetworkTableInstance.getDefault().getEntry(PREFIX + key).setDouble(value);
      } catch (Throwable ignore) {}
    }
  }

  public static void putBoolean(String key, boolean value) {
    try {
      SmartDashboard.putBoolean(PREFIX + key, value);
    } catch (Throwable t) {
      try {
        NetworkTableInstance.getDefault().getEntry(PREFIX + key).setBoolean(value);
      } catch (Throwable ignore) {}
    }
  }
}
