// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Spoon;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LoadingSpoon;

public class SpoonExtendAndRetract extends CommandBase {

  private LoadingSpoon loadingSpoon;

  /** Creates a new SpoonExtendAndRetract. */
  public SpoonExtendAndRetract(LoadingSpoon loadingSpoon) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.loadingSpoon = loadingSpoon;
    addRequirements(loadingSpoon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    loadingSpoon.toggle();

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
