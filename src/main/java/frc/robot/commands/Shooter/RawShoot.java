// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;

public class RawShoot extends CommandBase {

  private Shooter shooter;
  private double botSpeed, topSpeed;

  /** Creates a new RawShoot. */
  public RawShoot(Shooter shooter, double botSpeed, double topSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    addRequirements(this.shooter);

    this.botSpeed = botSpeed;
    this.topSpeed = topSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooter.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    shooter.setMotorSpeeds(ShooterConstants.BOT_SPEED, ShooterConstants.TOP_SPEED);

    SmartDashboard.putNumber("bot actual raw speed", shooter.getBotVelocity());
    SmartDashboard.putNumber("top actual raw speed", shooter.getTopVelocity());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
