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
import frc.robot.subsystems.Arm;

public class ArmPID extends CommandBase {

  private Arm arm;
  private double armDistance;
  private double armMargin;
  private double armError;

  /** Creates a new climberArmPID. */
  public ArmPID(Arm arm, double armDistance, double armMargin) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    addRequirements(arm);

    this.armDistance = armDistance;
    this.armMargin = armMargin;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    arm.setArmkP(ClimberConstants.kParm);
    arm.setArmkI(ClimberConstants.kIarm);
    arm.setArmkD(ClimberConstants.kDarm);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    arm.setArmOutputRange();
    arm.setArmSetPoint(armDistance);

    armError = Math.abs(armDistance - arm.getArmEncoder());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if ((armDistance > 0) && arm.isAReturnLimit()) {
      arm.resetArmEncoder();
      return true;
    } else if ((armDistance < 0) && arm.isAExtendLimit()) {
      return true;
    } else {
      return false;
    }

  }
}
