// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class WPI_Turn_PID extends CommandBase {
  private final Drive drive;
  private final PIDController pidController;

  /** Creates a new WPI_Turn_PID. */
  public WPI_Turn_PID(Drive drive, double setpoint) {
    this.drive = drive;
    this.pidController = new PIDController(DriveConstants.turnkP, 0, 0);
    pidController.setSetpoint(setpoint);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset();
    drive.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftSpeed = pidController.calculate(drive.getLeftDistance());
    double rightSpeed = pidController.calculate(-drive.getRightDistance());
    drive.setLeftSpeed(leftSpeed);
    drive.setRightSpeed(-rightSpeed);
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
