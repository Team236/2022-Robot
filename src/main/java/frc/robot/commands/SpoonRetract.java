// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LoadingSpoon;

public class SpoonRetract extends CommandBase {

  private LoadingSpoon retractSpoon;

  /** Creates a new SolenoidForward. */
  public SpoonRetract(LoadingSpoon retractSpoon) {
   this.retractSpoon = retractSpoon;
    // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(this.retractSpoon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("spoon retract execute", "true");
    retractSpoon.reverse();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
