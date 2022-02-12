// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;
import frc.robot.subsystems.ClimberArms;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimberArmRe extends CommandBase {

  private ClimberArms arm;
  final boolean isInverted = true;
  /** Creates a new ClimberArm. */
  public ClimberArmRe(ClimberArms arm) {
    this.arm = arm;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.armRetract(isInverted);
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
