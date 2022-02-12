// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase; 
import frc.robot.subsystems.LoadingSpoon; 

public class SpoonExtend extends CommandBase {
    private LoadingSpoon extendSpoon;

  /** Creates a new SolenoidForward. */
  public SpoonExtend(LoadingSpoon extendSpoon) {
   this.extendSpoon = extendSpoon;
    // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(this.extendSpoon);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putString("initialize", "has been initialized");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putString("spoon extend execute", "true");
    extendSpoon.forward();
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
