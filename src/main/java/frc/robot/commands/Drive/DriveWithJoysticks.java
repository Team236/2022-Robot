// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drive;

public class DriveWithJoysticks extends CommandBase {

  private Drive drive;
  private Joystick leftStick, rightStick;
  private boolean isCube;
  private int pow = 1;
  private boolean isDeadzone = Constants.DriveConstants.IS_DEADZONE;
  
  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, Joystick leftStick, Joystick rightStick) {

    this.drive = drive;
    this.leftStick = leftStick;
    this.rightStick = rightStick;
    this.isCube = isCube;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (isCube) {
      this.pow = 3;
      this.isDeadzone = false;
    } else {
      this.pow = 1;
      this.isDeadzone = true;
    }
    this.pow = 1;
    this.isDeadzone = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

     // IS_DEADZONE determines whether joystick deadzone is considered
     if (this.isDeadzone) {
      drive.setLeftSpeedWithDeadzone(Math.pow(-leftStick.getY(), pow));
      drive.setRightSpeedWithDeadzone(Math.pow(-rightStick.getY(), pow));
    } else {
      drive.setLeftSpeed(Math.pow(-leftStick.getY(), pow));
      drive.setRightSpeed(Math.pow(-rightStick.getY(), pow));
    }

    SmartDashboard.putNumber("DWJ getLeftEncoder", drive.getLeftEncoder());
    SmartDashboard.putNumber("DWJ getRightEncoder", drive.getRightEncoder());
    SmartDashboard.putNumber("DWJ L Distance", drive.getLeftDistance());
    SmartDashboard.putNumber("DWJ R Distance", drive.getRightDistance());
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
