package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.ShooterConfig;

public class BallShooter extends SubsystemBase {
  private PIDController rotationPID;
  private PIDController flywheelPID;
  private SparkMax rotationMotor;
  private SparkMax flywheel;

  public BallShooter() {
    rotationMotor = new SparkMax(CANConfig.SHOOTER_ROTATION, MotorType.kBrushless);
    rotationMotor.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    flywheel = new SparkMax(CANConfig.SHOOTER_FLYWHEEL, MotorType.kBrushless);
    flywheel.configure(new SparkMaxConfig().inverted(false),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    rotationPID = ShooterConfig.ROTATION_PID;
    flywheelPID = ShooterConfig.FLYWHEEL_PID;
  }

  @Override
  public void periodic() {
    updateEntries();
  }

  private void updateEntries() {
    Constants.sendNumberToElastic("Flywheel Speed", flywheel.get(), 2);

    Constants.sendNumberToElastic("Flywheel P", ShooterConfig.FLYWHEEL_P, 2);
    Constants.sendNumberToElastic("Flywheel I", ShooterConfig.FLYWHEEL_I, 2);
    Constants.sendNumberToElastic("Flywheel D", ShooterConfig.FLYWHEEL_D, 2);

    flywheelPID.setP(SmartDashboard.getNumber("Flywheel P", 0));
    flywheelPID.setI(SmartDashboard.getNumber("Flywheel I", 0));
    flywheelPID.setD(SmartDashboard.getNumber("Flywheel D", 0));
  }

  public void setFlywheelSpeed(double speed) {
    flywheel.set(speed);
  }

  public Command startFlywheel() {
    return new InstantCommand(() -> setFlywheelSpeed(ShooterConfig.FLYWHEEL_SPEED), this);
  }

  public Command stopFlywheel() {
    return new InstantCommand(() -> setFlywheelSpeed(0), this);
  }

  public PIDController getRotationController() {
    return rotationPID;
  }

  public PIDController getFlywheelController() {
    return flywheelPID;
  }

  public void rotateTo(float rad) {
    rotationPID.setSetpoint(rad);
  }
}