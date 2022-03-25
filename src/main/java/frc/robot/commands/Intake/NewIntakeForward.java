// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class NewIntakeForward extends CommandBase {

  private Intake intake;
  private double intakeSpeed, feedSpeed;
  
  /** Creates a new NewIntakeForward. */
  public NewIntakeForward(Intake intake, double intakeSpeed, double feedSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSpeed = intakeSpeed;
    this.feedSpeed = feedSpeed;
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

    if (intake.getFeederCount() == 0) {
      intake.setIntakeSpeed(intakeSpeed);
      intake.setFirstFeedSpeed(feedSpeed);
    } else {
      intake.setIntakeSpeed(intakeSpeed);
      intake.setFirstFeedSpeed(0);
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
    
    // if intake count = 1, keep intake running
    // if intake count = 2 and feed count = 1, stop intake and retract intake
    // else, keep intake running

    if (intake.getIntakeCount() == 1) {
      return false;
    } else if ((intake.getIntakeCount() == 2) && (intake.getFeederCount() == 1)) {
      intake.retract();
      intake.resetFeedCounter();
      return true;
    } else {
      return false;
    }


    // overall logic: 
    // turn on intake motor and first feeder motor
    // when feed counter = 1, turn off first feed motor
    // when intake counter = 1 and feed count = 1, turn off intake motor and retract intake
    // when intake counter = 2, turn off intake motor and retract intake

    // questions:
    // need to keep the color sensor?
    // when / where reset counters?
  }
}
