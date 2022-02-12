// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Mast;

public class MastIdle extends CommandBase {
  private Mast idle;
  /** Creates a new MastIdle. */
  public MastIdle(Mast idle) {
    this.idle = idle;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.idle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    idle.stop();
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
