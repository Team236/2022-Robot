// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Climber;

public class ArmPID extends CommandBase {

  private Climber climber;
  private double armDistance;
  private double armMargin;
  private double armError;

  /** Creates a new climberArmPID. */
  public ArmPID(Climber climber, double armDistance, double armMargin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    addRequirements(climber);

    this.armDistance = armDistance;
    this.armMargin = armMargin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    climber.resetEncoders();
    climber.setArmkP(ClimberConstants.kParm);
    climber.setArmkI(ClimberConstants.kIarm);
    climber.setArmkD(ClimberConstants.kDarm);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    climber.setArmOutputRange();
    climber.setArmSetPoint(armDistance);

    armError = Math.abs(armDistance - climber.getArmDistance());

    SmartDashboard.putNumber("PID arm revs", climber.getArmEncoder());
    SmartDashboard.putNumber("PID arm distance", climber.getArmDistance());
    SmartDashboard.putNumber("PID kParm", ClimberConstants.kParm);
    SmartDashboard.putNumber("PID arm setpoint1", armDistance);
    SmartDashboard.putNumber("PID arm ERROR", armError
);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if ((armDistance > 0) && !climber.isAExtendLimit()) {
    return true;
    } else if ((armDistance < 0) && !climber.isAReturnLimit()) {
      return true;
    } else {
      return false;
    }

  }
}
