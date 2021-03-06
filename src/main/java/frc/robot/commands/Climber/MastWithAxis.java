// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Mast;

public class MastWithAxis extends CommandBase {
  private Mast mast;
  private Joystick controller;
  private double speed;

  /** Creates a new ClimbWithAxis. */
  public MastWithAxis(Mast mast, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mast = mast;
    this.controller = controller;
    addRequirements(mast);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mast.setMastSpeed(-controller.getRawAxis(1));
    speed = -controller.getRawAxis(1);
    // axis 1 is the left joystick thingy on the logitech controller
    SmartDashboard.putNumber("mast speed", speed);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mast.mastStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((speed > 0.008) && mast.isMExtendLimit()) {
      // if mast is going up and top limit is triggered
      // the 0.008 is because when the axis is at rest, it reads 0.0078125 so doing speed > 0.008 acts as a deadzone
      return true;
    } else if ((speed < 0) && mast.isMReturnLimit()) {
      mast.resetMastEncoder();
      return true;
    } else {
      return false;
    }
  }
}
