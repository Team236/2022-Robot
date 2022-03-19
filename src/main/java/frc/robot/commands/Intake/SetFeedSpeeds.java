// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetFeedSpeeds extends CommandBase {

  private Intake intake;
  private double firstSpeed, secondSpeed;

  /** Creates a new SetFeedSpeeds. */
  public SetFeedSpeeds(Intake intake, double firstSpeed, double secondSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.firstSpeed = firstSpeed;
    this.secondSpeed = secondSpeed;
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setFirstFeedSpeed(firstSpeed);
    intake.setSecondFeedSpeed(secondSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopFirstFeed();
    intake.stopSecondFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
