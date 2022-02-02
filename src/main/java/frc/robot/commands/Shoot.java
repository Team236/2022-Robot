// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {

  private Shooter shooterSub;
  private double botSpeed, topSpeed;

  /** Creates a new Shoot. */
  public Shoot(Shooter shooterSub, double botSpeed, double topSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSub = shooterSub;
    addRequirements(this.shooterSub);

    this.botSpeed = botSpeed;
    this.topSpeed = topSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSub.resetEncoders();

    shooterSub.setP(Constants.ShooterConstants.kPShoot);
    shooterSub.setI(ShooterConstants.kIShoot);
    shooterSub.setD(ShooterConstants.kDShoot);
    shooterSub.setFF(ShooterConstants.kFFShoot);
    shooterSub.setOutputRange();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.setBotSetPoint(botSpeed);
    shooterSub.setTopSetPoint(topSpeed);

    SmartDashboard.putNumber("bot shoot setpoint", botSpeed);
    SmartDashboard.putNumber("top shoot setpoint", topSpeed);
    SmartDashboard.putNumber("bot actual speed", shooterSub.getBotVelocity());
    SmartDashboard.putNumber("top actual speed", shooterSub.getTopVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
