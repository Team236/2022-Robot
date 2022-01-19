// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class DriveWithPID extends CommandBase {

  private Drive drive;
  private double dist;
  private double margin;
  private double error;
  private double kP, kI, kD, kIz, kF;

  /** Creates a new DriveWithPID. */
  public DriveWithPID(Drive drive, double dist, double margin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);

    this.dist = dist;
    this.margin = margin;

    this.kP = Constants.DriveConstants.kP;
    this.kI = Constants.DriveConstants.kI;
    this.kD = Constants.DriveConstants.kD;
    this.kIz = Constants.DriveConstants.kIz;
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
    drive.setkIz(this.kIz);
    drive.setkF(this.kF);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    drive.setOutputRange(DriveConstants.MIN_OUTPUT, DriveConstants.MAX_OUTPUT);
    drive.setSetPoint(dist);

    // calculates error in inches
    error = Math.abs(dist - drive.getLeftDistance());

    // SmartDashboard.putNumber("Drive Setpoint", dist);
    SmartDashboard.putNumber("Drive Error", error);
    SmartDashboard.putNumber("Drive L Encoder", drive.getLeftEncoder());
    SmartDashboard.putNumber("Drive R Encoder", drive.getRightEncoder());
    SmartDashboard.putNumber("Drive L Distance", drive.getLeftDistance());
    SmartDashboard.putNumber("Drive R Distance", drive.getRightDistance());
    SmartDashboard.putNumber("Drive kP", this.kP);
    SmartDashboard.putNumber("Drive setpoint", this.dist);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    System.out.println("DriveWithPID interrupted");
    drive.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // ends command when the error is less than the margin
    boolean isDistMargin = error < margin;

    // SmartDashboard.putString("Is Finished", "is finished");

    return isDistMargin;
  }
}
