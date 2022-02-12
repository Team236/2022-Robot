// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

public class DriveWithPID extends CommandBase {

  private Drive drive;
  private double dist;
  private double margin;
  private double error;
  // private double kP, kI, kD, kF;
  // private SparkMaxPIDController leftPID, rightPID;

  /** Creates a new DriveWithPID. */
  public DriveWithPID(Drive drive, double dist, double margin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);

    this.dist = dist;
    this.margin = margin;

    // kP = Constants.DriveConstants.kP;
    // kI = Constants.DriveConstants.kI;
    // kD = Constants.DriveConstants.kD;
    // kF = Constants.DriveConstants.kF;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // error = dist;

    SmartDashboard.putNumber("PID Error in init", error);

    System.out.println("drive with PID initialized");
    drive.resetEncoders();

    // sets PIDF values to the values defined in DriveConstants
    // leftPID.setP(DriveConstants.kP);
    // rightPID.setP(DriveConstants.kP);
    // leftPID.setI(DriveConstants.kI);
    // rightPID.setI(DriveConstants.kI);
    // leftPID.setD(DriveConstants.kD);
    // rightPID.setD(DriveConstants.kD);

    drive.setkP(DriveConstants.kP);
    drive.setkI(DriveConstants.kI);
    drive.setkD(DriveConstants.kD);
    drive.setkF(DriveConstants.kF);

    // drive.setRawSetPoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // drive.setOutputRange(DriveConstants.MIN_OUTPUT, DriveConstants.MAX_OUTPUT);
    drive.setSetPoint(dist);

    // calculates error in inches
    error = Math.abs(dist - drive.getLeftDistance());
    // error = Math.abs(10 - drive.getLeftEncoder());

    SmartDashboard.putNumber("PID L Encoder", drive.getLeftEncoder());
    SmartDashboard.putNumber("PID R Encoder", drive.getRightEncoder());
    SmartDashboard.putNumber("PID L Distance", drive.getLeftDistance());
    SmartDashboard.putNumber("PID R Distance", drive.getRightDistance());
    // SmartDashboard.putNumber("PID kP", kP);
    SmartDashboard.putNumber("PID setpoint", dist);
    SmartDashboard.putNumber("PID Error", error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    System.out.println("drive with PID interrupted");
    SmartDashboard.putNumber("PID Error in end", error);

    // drive.resetEncoders();
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // System.out.println("drive with PID finished");


    // ends command when the error is less than the margin
    boolean isDistMargin = error < margin;
    SmartDashboard.putBoolean("isFinished", isDistMargin);

    return isDistMargin;
  }
}
