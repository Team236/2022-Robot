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
//  private int myCount = 1;
  
  /** Creates a new SetIntakeSpeed. */
  public IntakeForward(Intake intake, double speed) {

    this.intake = intake;
    this.speed = speed;
    // myCount = 1;
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

    SmartDashboard.putNumber("count in intake execute", intake.getBallCount());

    intake.setSpeed(speed);

    // if ((intake.getBallCount() == 0) && (!colorSensor.isBallInSpoon())) {
    //   intake.setSpeed(speed);
    // } else if ((intake.getBallCount() == 0) && (colorSensor.isBallInSpoon())) {
    //   intake.setSpeed(speed);
    // } else if ((intake.getBallCount() == 1) && (colorSensor.isBallInSpoon())) {
    //   intake.setSpeed(0);
    //   intake.resetCounter();
    // } else if ((intake.getBallCount() == 1) && (!colorSensor.isBallInSpoon())) {
    //   intake.setSpeed(speed);
    // }

    // if (intake.getBallCount() == 1) {
    //   intake.setSpeed(0);
    // } else if (intake.getBallCount() == 2) {
    //   intake.setSpeed(0);
    // } else if (intake.getBallCount() == 0) {
    //   intake.setSpeed(speed);
    // }

    // if ball is in intake and ball is NOT in spoon, then keep intake running
    // if ((intake.getBallCount() == (previousCount + 1)) && (colorSensor.getDistance() < 1000)) {
    //   intake.setSpeed(speed);
    // } 
    // // else if ball is in intake and ball IS in spoon, then stop intake
    // else if ((intake.getBallCount() == previousCount +1) && (colorSensor.getDistance() > 1100)) {
    //   intake.setSpeed(0);
    // } 
    // // else, keep intake running
    // else {
    //   intake.setSpeed(speed);
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if ((intake.getBallCount() == 0) && (!intake.isBallInSpoon())) { // no ball in intake and no ball in spoon
      return false;
    } else if ((intake.getBallCount() == 0) && (intake.isBallInSpoon())) { // no ball in intake and ball in spoon
      return false;
    } else if ((intake.getBallCount() == 1) && (!intake.isBallInSpoon())) { // ball in intake and no ball in spoon
      return false;
    } else if ((intake.getBallCount() >= 1) && (intake.isBallInSpoon())) { // ball in intake and ball in spoon
      intake.resetCounter();
      return true;
    } else {
      return false;
    }
  }
}

