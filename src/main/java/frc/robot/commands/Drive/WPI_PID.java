// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class WPI_PID extends CommandBase {
  private final Drive drive;
  private final PIDController leftPidController, rightPidController;

  /** Creates a new SeparatePID. */
  public WPI_PID(Drive drive, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.leftPidController = new PIDController(DriveConstants.kPLeft, DriveConstants.kI, DriveConstants.kD);
    this.rightPidController = new PIDController(DriveConstants.kPRight, DriveConstants.kI, DriveConstants.kD);
    leftPidController.setSetpoint(setPoint);
    rightPidController.setSetpoint(setPoint);
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rightPidController.reset();
    leftPidController.reset();
    drive.resetEncoders();
    drive.closedRampRate();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = leftPidController.calculate(drive.getLeftDistance());
    double rightSpeed = rightPidController.calculate(drive.getRightDistance());
    drive.setLeftSpeed(leftSpeed);
    drive.setRightSpeed(rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
