// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class NewIntakeForward extends CommandBase {

  private Intake intake;
  
  /** Creates a new NewIntakeForward. */
  public NewIntakeForward(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.resetIntkCounter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (intake.isBallInSpoon()) {
      intake.setIntakeSpeed(IntakeConstants.FORWARD_SPEED);
      intake.setFirstFeedSpeed(0);
    } else {
      intake.setIntakeSpeed(IntakeConstants.FORWARD_SPEED);
      intake.setFirstFeedSpeed(IntakeConstants.FIRST_FEED_SPEED);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    intake.stopFirstFeed();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ((intake.getIntakeCount() == 1) && (!intake.isBallInSpoon())) {
      intake.resetIntkCounter();
      return false;
    } else if ((intake.getIntakeCount() == 1) && (intake.isBallInSpoon())) {
      intake.retract();
      return true;
    } else if (intake.getIntakeCount() == 2) {
      intake.retract();
      return true;
    } else {
      return false;
    }
  }
}
