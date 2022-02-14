// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class AnotherOne extends CommandBase {

  private Drive drive;

  /** Creates a new AnotherOne. */
  public AnotherOne(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.resetEncoders();
    drive.setkP(0.0015);
    drive.setkI(0);
    drive.setkD(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setSetPoint(20);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    drive.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
