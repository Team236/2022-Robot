// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.ControllerConstants.LogitechF310;
import frc.robot.subsystems.Climber;

public class MastWithAxis extends CommandBase {
  private Climber climber;
  private Joystick controller;
  private double speed;

  /** Creates a new ClimbWithAxis. */
  public MastWithAxis(Climber climber, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.controller = controller;
    addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // climber.setSpeed(controller.getRawAxis((ControllerConstants.LogitechF310.AxesController.LEFT_Y)/2)); // speed might have to be negative
    // climber.setMastSpeed(controller.getRawAxis((ControllerConstants.LogitechF310.AxesController.LEFT_Y)));
    climber.setMastSpeed(controller.getY());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.mastStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if ((speed > 0) && !climber.isMExtendLimit()) {
    //   return true;
    // } else if ((speed < 0) && !climber.isMReturnLimit()) {
    //   return true;
    // } else {
    //   return false;
    // }
    return false;
  }
}
