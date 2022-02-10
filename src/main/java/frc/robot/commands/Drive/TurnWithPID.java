// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class TurnWithPID extends CommandBase {

  private Drive drive;
  private double dist, margin, error;
  private double kP, kI, kD, kF;
  /** Creates a new TurnWithPID. */
  public TurnWithPID(Drive turnPidDrive, double dist, double margin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = turnPidDrive;
    addRequirements(this.drive);

    this.dist = dist;
    this.margin = margin;

    this.kP = Constants.DriveConstants.turnkP;
    this.kI = Constants.DriveConstants.kI;
    this.kD = Constants.DriveConstants.kD;
    this.kF = Constants.DriveConstants.kF;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive.resetEncoders();
    
    // sets PIDF values to the values defined in DriveConstants
    drive.setkP(this.kP);
    drive.setkI(this.kI);
    drive.setkD(this.kD);
    drive.setkF(this.kF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drive.setOutputRange(DriveConstants.MIN_OUTPUT, DriveConstants.MAX_OUTPUT);
    drive.setTurnSetPoint(dist);

    // calculates error in inches
    error = Math.abs(dist - drive.getLeftDistance());

    SmartDashboard.putNumber("PID Turn L Encoder", drive.getLeftEncoder());
    SmartDashboard.putNumber("PID Turn R Encoder", drive.getRightEncoder());
    SmartDashboard.putNumber("PID Turn L Distance", drive.getLeftDistance());
    SmartDashboard.putNumber("PID Turn R Distance", drive.getRightDistance());
    SmartDashboard.putNumber("PID Turn kP", kP);
    SmartDashboard.putNumber("PID Turn setpoint", dist);
    SmartDashboard.putNumber("PID Turn Error", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // ends command when the error is less than the margin
    boolean isDistMargin = error < margin;

    return false;
  }
}
