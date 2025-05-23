// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.BallShooter;

/** An example command that uses an example subsystem. */
public class ShootBall extends SequentialCommandGroup {
  public ShootBall(BallShooter ballShooter) {
    addRequirements(ballShooter);
    addCommands(
        ballShooter.startFlywheel(),
        new WaitCommand(5),
        ballShooter.stopFlywheel());
  }
}
