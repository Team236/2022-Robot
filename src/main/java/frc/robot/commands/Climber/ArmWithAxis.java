package frc.robot.commands.Climber;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class ArmWithAxis extends CommandBase {
  private Arm arm;
  private Joystick controller;
  private double speed;

  /** Creates a new ArmWithAxis. */
  public ArmWithAxis(Arm arm, Joystick controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.controller = controller;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // down/bad moves car closer to top hooks, negative speed, red, retract
    // up/good moves car closer to Neo motor, positive speed, green, extend
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmSpeed(-controller.getRawAxis(3));
    speed = -controller.getRawAxis(3);
    // SmartDashboard.putNumber("arm speed", speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((speed > 0.008) && arm.isAReturnLimit()) {
      arm.resetArmEncoder();
      return true;
    } else if ((speed < 0) && arm.isAExtendLimit()) {
        // the 0.008 is because when the axis is at rest, it reads -0.0078125 so doing speed < 0.008 acts as a deadzone
      return true;
    } else {
      return false;
    }
  }
}
