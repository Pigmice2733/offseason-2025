// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double AXIS_THRESHOLD = 0.1;

  public class CANConfig {
    public static final int SHOOTER_ROTATION = 0;
    public static final int SHOOTER_FLYWHEEL = 0;
  }

  public static class DrivetrainConfig {
    public static final double MAX_DRIVE_SPEED = 10.0; // m/s
    public static final double MAX_TURN_SPEED = 10.0; // rad/s
    public static final double SLOWMODE_FACTOR = 0.1;

    public static final PIDConstants DRIVE_PID = new PIDConstants(4.0, 0.0, 1.3);

    public static final PIDConstants TURN_PID = new PIDConstants(2.5, 0.0, 0);
  }

  public class ShooterConfig {
    public static final PIDController FLYWHEEL_PID = new PIDController(
        ShooterConfig.FLYWHEEL_P,
        ShooterConfig.FLYWHEEL_I,
        ShooterConfig.FLYWHEEL_D);
    public static final double FLYWHEEL_P = 0.0;
    public static final double FLYWHEEL_I = 0.0;
    public static final double FLYWHEEL_D = 0.0;

    public static final PIDController ROTATION_PID = new PIDController(
        ShooterConfig.ROTATION_P,
        ShooterConfig.ROTATION_I,
        ShooterConfig.ROTATION_D);
    public static final double ROTATION_P = 0.0;
    public static final double ROTATION_I = 0.0;
    public static final double ROTATION_D = 0.0;

    public static final double FLYWHEEL_SPEED = 1.0;
  }

  public static void sendNumberToElastic(String name, double num, double places) {
    double newNum = Math.round(num * Math.pow(10, places)) / Math.pow(10, places);
    SmartDashboard.putNumber(name, newNum);
  }

  public static void sendBooleanToElastic(String name, boolean val) {
    SmartDashboard.putBoolean(name, val);
  }
}
