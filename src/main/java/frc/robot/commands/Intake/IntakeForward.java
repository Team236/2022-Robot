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
  
  /** Creates a new SetIntakeSpeed. */
  public IntakeForward(Intake intake, double speed) {

    this.intake = intake;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
@Override
  public void initialize() {
    intake.resetIntkCounter();
    SmartDashboard.putBoolean("FULL", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakeSpeed(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    SmartDashboard.putBoolean("FULL", true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (intake.getIntakeCount() == 0) { // no ball in intake
      return false;
    } else if ((intake.getIntakeCount() == 1) && (!intake.isBallInSpoon())) { // ball in intake and no ball in spoon
      intake.resetIntkCounter();
      return false;
    } else if ((intake.getIntakeCount() == 1) && (intake.isBallInSpoon())) { // ball in intake and ball in spoon
      return true;
    } else if (intake.getIntakeCount() == 2) {
      return true;
    } else {
      return false;
    }
  }
}
