// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RawShoot extends CommandBase {

  private Shooter shooter;
  private double botSpeed, topSpeed;

  /** Creates a new RawShoot. */
  public RawShoot(Shooter shooter, double botSpeed, double topSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setBotRawSpeed(botSpeed);
    shooter.setTopRawSpeed(topSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
