// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeExtend extends CommandBase {

  private Intake intake;
  private boolean isAuto;

  /** Creates a new IntakeExtend. */
  public IntakeExtend(Intake intake, boolean isAuto) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.isAuto = isAuto;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    intake.extend();
    if (!isAuto) {
      intake.resetFeedCounter();
    }
    intake.resetIntkCounter();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
