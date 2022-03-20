// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class TargetBall extends CommandBase {

  private Drive drive;
  private double proportional, integral, derivative;
  private double speed, error, errorT, lastError;
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double integralActiveZone = 5;

  /** Creates a new TargetBall. */
  public TargetBall(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double angle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    error = angle;

    // proportional
    proportional = error * kP;

    // integral
    if ((error < integralActiveZone) && (error > -integralActiveZone)) {
      errorT += error;
    } else {
      errorT = 0;
    }

    if (errorT > (50 / kI)) {
      errorT = 50 / kI;
    }

    integral = errorT * kI;

    // derivative
    derivative = (error - lastError) * kD;

    if (error == 0) {
      derivative = 0;
    }

    lastError = error;

    speed = (proportional + integral + derivative);

    drive.setLeftSpeed(speed);

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
