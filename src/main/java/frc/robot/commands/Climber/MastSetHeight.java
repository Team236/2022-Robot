// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Mast;

public class MastSetHeight extends CommandBase {
  private Mast mast;
  private double mastHeight;
  private double speed;
  private double encoder;

  /** Creates a new MastSetHeight. */
  public MastSetHeight(Mast mast, double mastHeight, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mast = mast;
    this.mastHeight = mastHeight;
    this.speed = speed;
    addRequirements(mast);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mast.setMastSpeed(speed);
    encoder = mast.getMastEncoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mast.mastStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (encoder == mastHeight) {
      return true;
    } else if ((speed > 0) && !mast.isMExtendLimit()) {
      return true;
    } else if ((speed < 0) && !mast.isMReturnLimit()) {
      mast.resetMastEncoder();
      return true;
    } else {
      return false;
    }
  }
}
