// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DashboardPID extends CommandBase {

  private Drive drive;
  private double dist, margin, error;
  private double kP, kI, kD;
  private double p, i, d;
  // private double lastp, lasti, lastd, lastf;

  /** Creates a new DashboardPID. */
  public DashboardPID(Drive drive, double dist, double margin) {
    /* *** PID position control using SparkMax PID control is NOT reliable -- better to use WPILib PID control *** */

  
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);

    this.dist = dist;
    this.margin = margin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive.resetEncoders();

    // sets initial PID values to 0
    kP = 0; 
    kI = 0; 
    kD = 0;

    // sets the Spark PID controllers to the PID values
    drive.setkP(this.kP);
    drive.setkI(this.kI);
    drive.setkD(this.kD);
    
    // puts the editable PIDF values on the dashboard
    SmartDashboard.putNumber("edit kP", kP);
    SmartDashboard.putNumber("edit kI", kI);
    SmartDashboard.putNumber("edit kD", kD);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // sets variables equal the smardashboard editable PID values
    p = SmartDashboard.getNumber("edit kP", 0);
    i = SmartDashboard.getNumber("edit kI", 0);
    d = SmartDashboard.getNumber("edit kD", 0);

    // if dashboard PID values are not equal to the set values in init, then set the values equal to the dashboard values
    if((p != kP)) {
      drive.setkP(p); kP = p; 
    }
    if((i != kI)) { 
      drive.setkI(i); kI = i; 
    }
    if((d != kD)) { 
      drive.setkD(d); kD = d; 
    }

    drive.setOutputRange(Constants.DriveConstants.MIN_OUTPUT, Constants.DriveConstants.MAX_OUTPUT);
    drive.setSetPoint(dist);

    // calculates error in inches
    error = Math.abs(dist - drive.getAvgDistance());

    SmartDashboard.putNumber("PID Dash setPoint", dist);
    SmartDashboard.putNumber("PID Dash error", error);
    SmartDashboard.putNumber("PID Dash L distance", drive.getLeftDistance());
    SmartDashboard.putNumber("PID Dash R distance", drive.getRightDistance());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drive.resetEncoders();
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // boolean isDistMargin = error < margin;
    // SmartDashboard.putBoolean("isFinished", isFinished());

    return false;
  }
}
