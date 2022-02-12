// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Mast;

public class MastRetract extends CommandBase {

  private Mast mastRe;
  final boolean isInverted = true;
  /** Creates a new Mast. */
  public MastRetract(Mast mastRe) {
    this.mastRe = mastRe;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.mastRe);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mastRe.mastRetract(isInverted);
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
