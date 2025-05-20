package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BallShooterConfig;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.PIDConfig;

public class BallShooter extends SubsystemBase {
  private PIDController rotationPID;
  @SuppressWarnings("unused")
  private PIDController flyWheelPID;
  private SparkMax rotationMotor;
  private SparkMax flyWheel;

  public BallShooter() {
    rotationMotor = new SparkMax(CANConfig.BALL_SHOOTER_ROTATION, MotorType.kBrushless);
    rotationMotor.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    flyWheel = new SparkMax(CANConfig.BALL_SHOOTER_FLY_WHEEL, MotorType.kBrushless);
    flyWheel.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    flyWheelPID = PIDConfig.FLY_WHEEL;
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("FlyWheel Speed", flyWheel.get(), 2);
  }

  public void setFlyWheelSpeed(double speed) {
    flyWheel.set(speed);
  }

  public Command stopFlyWheel() {
    return new InstantCommand(() -> setFlyWheelSpeed(0), this);
  }

  public Command startFlyWheel() {
    return new InstantCommand(() -> setFlyWheelSpeed(BallShooterConfig.FLYWHEEL_SPEED), this);
  }

  public void rotateTo(float rad) {
    rotationPID.setSetpoint(rad);
  }

  public PIDController getController() {
    return null;
  }
}