// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSensor;

public class GetColorSensor extends CommandBase {

  private ColorSensor colorSensor;

  /** Creates a new GetColorSensor. */
  public GetColorSensor(ColorSensor colorSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.colorSensor = colorSensor;
    addRequirements(this.colorSensor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    colorSensor.getAllianceColor();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    colorSensor.getColor();
    colorSensor.publishAllianceColor();
    colorSensor.getBlue();
    colorSensor.getRed();
    colorSensor.isBallMine();
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
