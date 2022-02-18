// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeForward extends CommandBase {

  private Intake intake;
  private double speed;
  private int myCount;
  
  /** Creates a new SetIntakeSpeed. */
  public IntakeForward(Intake intake, double speed) {

    this.intake = intake;
    this.speed = speed;
    myCount = 1;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (myCount == 1) {
      intake.resetCounter();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("count in intake execute", intake.getBallCount());

    intake.setSpeed(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    myCount = myCount +1;

    if (intake.getBallCount() == 0) { // no ball in intake
      return false;
     } else if ((intake.getBallCount() == 1) && (!intake.isBallInSpoon())) { // ball in intake and no ball in spoon
      intake.resetCounter();
      return false;
    } else if ((intake.getBallCount() == 1) && (intake.isBallInSpoon())) { // ball in intake and ball in spoon
      return true;
    } else {
      return false;
    }
  }
}
