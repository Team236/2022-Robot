// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class ArmWithAxis extends CommandBase {
  private Climber climber;
  private Joystick controller;
  private double speed;

  /** Creates a new ArmWithAxis. */
  public ArmWithAxis(Climber climber, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.controller = controller;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setArmSpeed(-controller.getRawAxis(3)); //might be 4 or 3
    speed = -controller.getRawAxis(3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((speed > 0.008) && !climber.isAExtendLimit()) {
      // the 0.008 is because when the axis is at rest, it reads 0.0078125 so doing speed > 0.008 acts as a deadzone
      return true;
    } else if ((speed < 0) && !climber.isAReturnLimit()) {
      return true;
    } else {
      return false;
    }
  }
}