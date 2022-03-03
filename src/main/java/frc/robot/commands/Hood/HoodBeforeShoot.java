// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Hood;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Hood;

public class HoodBeforeShoot extends CommandBase {

  private Hood hood;
  private double botSpeed;

  /** Creates a new HoodBeforeShoot. */
  public HoodBeforeShoot(Hood hood, double botSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hood = hood;
    addRequirements(hood);
    this.botSpeed = botSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (botSpeed == ShooterConstants.LOW_HUB_BOT || botSpeed == ShooterConstants.HIGH_HUB_BOT) {
      hood.hoodRetract();
    } else {
      hood.hoodExtend();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
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
