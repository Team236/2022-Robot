// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HangarPID.ArmPID;

import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hangar;

public class ArmPID2 extends CommandBase {

  private Hangar hangarArm;
  private double armDISTANCE2;
  private double armMARGIN;
  private double armERROR1;

  /** Creates a new ArmPID2. */
  public ArmPID2(Hangar hangarArm, double armDISTANCE2, double armMARGIN, double kParm, double kIarm, double kDarm) {
   
    this.hangarArm = hangarArm;
    addRequirements(hangarArm);

    this.armDISTANCE2 = armDISTANCE2;
    this.armMARGIN = armMARGIN;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hangarArm.resetEncoders();

    hangarArm.setArmkP(Constants.Hanger.HangarPIDConstants.kParm);
    hangarArm.setArmkI(Constants.Hanger.HangarPIDConstants.kIarm);
    hangarArm.setArmkD(Constants.Hanger.HangarPIDConstants.kDarm);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangarArm.setArmOutputRange();
    hangarArm.setArmSetPoint2();

    armERROR1 = Math.abs(armDISTANCE2 - hangarArm.getArmDistance());

    SmartDashboard.putNumber("PID arm", hangarArm.getArmEncoder());
    SmartDashboard.putNumber("PID arm distance", hangarArm.getArmDistance());
    SmartDashboard.putNumber("PID kParm", Constants.Hanger.HangarPIDConstants.kParm);
    SmartDashboard.putNumber("PID arm setpoint2", armDISTANCE2);
    SmartDashboard.putNumber("PID arm ERROR", armERROR1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangarArm.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isArmMargin = armERROR1 < armMARGIN;
    SmartDashboard.putBoolean("arm PID finished", isArmMargin);
    
    return false;
  }
}
