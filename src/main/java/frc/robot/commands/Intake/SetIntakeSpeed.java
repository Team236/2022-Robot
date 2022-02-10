// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class SetIntakeSpeed extends CommandBase {

  private Intake intake;
  private double speed;
  
  /** Creates a new SetIntakeSpeed. */
  public SetIntakeSpeed(Intake intake, double speed) {

    this.intake = intake;
    this.speed = speed;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.resetCounter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setSpeed(speed);

    SmartDashboard.putNumber("intake counter", intake.getBallCount());
    
    if (intake.getBallCount() == 1) {
      intake.setSpeed(0);
    } else if (intake.getBallCount() == 2) {
      intake.setSpeed(0);
    } else if (intake.getBallCount() == 0) {
      intake.setSpeed(speed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
