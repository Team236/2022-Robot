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
  private double kP, kI, kD, kFF;
  private double p, i, d, ff;
  // private double lastp, lasti, lastd, lastf;

  /** Creates a new DashboardPID. */
  public DashboardPID(Drive drive, double dist, double margin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(this.drive);

    this.dist = dist;
    this.margin = margin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drive.resetEncoders();

    // sets initial PIDF values to 0
    kP = 0; 
    kI = 0; 
    kD = 0;
    kFF = 0;

    // sets the Spark PID controllers to the PIDF values
    drive.setkP(this.kP);
    drive.setkI(this.kI);
    drive.setkD(this.kD);
    drive.setkF(this.kFF);
    
    // puts the editable PIDF values on the dashboard
    SmartDashboard.putNumber("edit kP", kP);
    SmartDashboard.putNumber("edit kI", kI);
    SmartDashboard.putNumber("edit kD", kD);
    SmartDashboard.putNumber("edit Feed Fwd", kFF);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // gets the PIDF values from the dashboard
    p = SmartDashboard.getNumber("edit kP", 0);
    i = SmartDashboard.getNumber("edit kI", 0);
    d = SmartDashboard.getNumber("edit kD", 0);
    ff = SmartDashboard.getNumber("edit Feed Fwd", 0);

    // if dashboard PIDF values are not equal to the set values in init, then set the values equal to the dashboard values
    if((p != kP)) { drive.setkP(p); kP = p; }
    //  SmartDashboard.putNumber("set p", p);
    if((i != kI)) { drive.setkI(i); kI = i; }
    //  SmartDashboard.putNumber("set i", i);
    if((d != kD)) { drive.setkD(d); kD = d; }
    //  SmartDashboard.putNumber("set pd", d);
    if((ff != kFF)) { drive.setkF(ff); kFF = ff; }
    //  SmartDashboard.putNumber("set f", ff);

    drive.setOutputRange(Constants.DriveConstants.MIN_OUTPUT, Constants.DriveConstants.MAX_OUTPUT);
    drive.setSetPoint(dist);

    // calculates error in inches
    error = Math.abs(dist - drive.getLeftDistance());

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


    // finishes command when error is less than the margin
    boolean isDistMargin = error < margin;
    // SmartDashboard.putBoolean("isFinished", isFinished());

    return false;
  }
}
