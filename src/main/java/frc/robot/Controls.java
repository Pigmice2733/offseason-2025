package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivetrainConfig;

public class Controls {
  private CommandXboxController driver, operator;

  /** If a joystick is moved less than this, it will not register as movement */
  private double threshold = Constants.AXIS_THRESHOLD;

  private boolean slowmode;

  public Controls(CommandXboxController driver, CommandXboxController operator) {
    this.driver = driver;
    this.operator = operator;

    setSlowmode(false);
  }

  /**
   * Returns the left joystick's Y-axis value, corresponding to the robot's
   * X-direction speed. Inverted because positive joystick axis is down but
   * positive X-movement is forward.
   */
  public double getDriveSpeedX() {
    double joystickY = MathUtil.applyDeadband(driver.getLeftY(), threshold);

    return -1 * joystickY * DrivetrainConfig.MAX_DRIVE_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
  }

  /**
   * Returns the left joystick's X-axis value, corresponding to the robot's
   * Y-direction speed. Inverted because positive joystick axis is right but
   * positive Y-movement is left.
   */
  public double getDriveSpeedY() {
    double joystickX = MathUtil.applyDeadband(driver.getLeftX(), threshold);

    return -1 * joystickX * DrivetrainConfig.MAX_DRIVE_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
  }

  /**
   * Returns the right joystick's X-axis value, corresponding to the robot's
   * rotational speed. Inverted because positive joystick axis is right but
   * positive rotation is left.
   */
  public double getTurnSpeed() {
    double joystickTurn = MathUtil.applyDeadband(driver.getRightX(), threshold);

    return -1 * joystickTurn * DrivetrainConfig.MAX_TURN_SPEED * (slowmode ? DrivetrainConfig.SLOWMODE_FACTOR : 1);
  }

  /** Turns on slowmode if slowmode is off, turns it off it is on. */
  public Command toggleSlowMode() {
    return new InstantCommand(() -> setSlowmode(!getSlowmode()));
  }

  public Command rumbleCommand(double rumble) {
    return new InstantCommand(() -> {
      driver.setRumble(RumbleType.kBothRumble, rumble);
      operator.setRumble(RumbleType.kBothRumble, rumble);
    });
  }

  public Command rumbleCommand() {
    return rumbleCommand(0.599999d);
  }

  /**
   * Turns slowmode on or off.
   * 
   * @param slow whether slowmode is turned on or off
   */
  public void setSlowmode(boolean slow) {
    slowmode = slow;
    SmartDashboard.putBoolean("Slowmode", slowmode);
  }

  /**
   * @return Whether slowmode is active
   */
  public boolean getSlowmode() {
    return slowmode;
  }
}
