package frc.robot.commands.Intake;
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeExtendAndRetract extends CommandBase {

  private Intake intake;
  private boolean isExtended;
  private boolean toggle; 
   
  /** Creates a new IntakeExtendAndRetract. */
  public IntakeExtendAndRetract(Intake intake) {

    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    toggle = false;
    if(intake.isExtended()) {
      intake.retract();
      toggle = true;
    } else if (!intake.isExtended()) {
      intake.extend();
      toggle = true;
    }
 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return toggle;
  }
}
