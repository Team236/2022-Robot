// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

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
    /* *** sets RPMs of top and bottom shooter rollers using SparkMax PID velocity control -- 
    unlike Spark PID position control, velocity control is reliable and the command finishes normally *** */

    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSub = shooterSub;
    addRequirements(this.shooterSub);

    this.botSpeed = botSpeed;
    this.topSpeed = topSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    // shooterSub.restoreFactoryDefaults();
    shooterSub.resetEncoders();

    shooterSub.setP(ShooterConstants.kPBot, ShooterConstants.kPTop);
    shooterSub.setI(ShooterConstants.kIBot, ShooterConstants.kITop);
    shooterSub.setD(ShooterConstants.kDBot, ShooterConstants.kDTop);
    shooterSub.setFF(ShooterConstants.kFFBot, ShooterConstants.kFFTop);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooterSub.setOutputRange();

    shooterSub.setBotSetPoint(botSpeed);
    shooterSub.setTopSetPoint(topSpeed);
    
    SmartDashboard.putNumber("bot wheel setpoint", ((botSpeed * 32) / 18));
    SmartDashboard.putNumber("top wheel setpoint", ((topSpeed * 32) / 18));
    SmartDashboard.putNumber("bot shoot setpoint", shooterSub.getBotVelocity());
    SmartDashboard.putNumber("top shoot setpoint", shooterSub.getTopVelocity());
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
