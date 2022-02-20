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

public class ArmPID extends CommandBase {

  private Hangar hangar;
  private double armDISTANCE;
  private double armMARGIN;
  private double armERROR;

  /** Creates a new HangarArmPID. */
  public ArmPID(Hangar hangar, double armDISTANCE, double armMARGIN) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hangar = hangar;
    addRequirements(hangar);

    this.armDISTANCE = armDISTANCE;
    this.armMARGIN = armMARGIN;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    hangar.resetEncoders();

    hangar.setArmkP(Constants.Hanger.HangarPIDConstants.kParm);
    hangar.setArmkI(Constants.Hanger.HangarPIDConstants.kIarm);
    hangar.setArmkD(Constants.Hanger.HangarPIDConstants.kDarm);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    hangar.setArmOutputRange();
    hangar.setArmSetPoint(armDISTANCE);

    armERROR = Math.abs(armDISTANCE - hangar.getArmDistance());

    SmartDashboard.putNumber("PID arm revs", hangar.getArmEncoder());
    SmartDashboard.putNumber("PID arm distance", hangar.getArmDistance());
    SmartDashboard.putNumber("PID kParm", Constants.Hanger.HangarPIDConstants.kParm);
    SmartDashboard.putNumber("PID arm setpoint1", armDISTANCE);
    SmartDashboard.putNumber("PID arm ERROR", armERROR);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangar.armStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

   // boolean isArmMargin = armERROR < armMARGIN;
   // SmartDashboard.putBoolean("arm PID finished", isArmMargin);
    
    return false;
  }
}
