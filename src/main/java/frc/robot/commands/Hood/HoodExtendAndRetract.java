// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hood;

public class HoodExtendAndRetract extends CommandBase {

  private Hood hood;
  private boolean hoodIsExtended;
  private boolean toggle;
  /** Creates a new HoodExtendAndRetract. */
  public HoodExtendAndRetract(Hood hood) {
    this.hood = hood;
   addRequirements(hood);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    toggle = false;

    if (hood.hoodIsExtended()) {
      hood.hoodRetract();
      toggle = true;
    } else if (!hood.hoodIsExtended()) {
      hood.hoodExtend();
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
