// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.HangarPID.MastPID;

import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hangar;

public class MastPIDUp extends CommandBase {

  private Hangar hangarMast;
  private double mastDISTANCE1;
  private double mastMARGIN;
  private double mastERROR;
  
  /** Creates a new HangerMastPID. */
  public MastPIDUp(Hangar hangarMast, double mastDISTANCE1, double mastMARGIN) {

    this.hangarMast = hangarMast;
    addRequirements(hangarMast);
    // Use addRequirements() here to declare subsystem dependencies.

    this.mastDISTANCE1 = mastDISTANCE1;
    this.mastMARGIN = mastMARGIN;
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    hangarMast.resetEncoders();
    hangarMast.setMastkP(Constants.Hanger.HangarPIDConstants.kPmast);
    hangarMast.setMastkI(Constants.Hanger.HangarPIDConstants.kImast);
    hangarMast.setMastkD(Constants.Hanger.HangarPIDConstants.kDmast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangarMast.setArmOutputRange();
    hangarMast.setMastSetPoint1();
    mastERROR = Math.abs(mastDISTANCE1 - hangarMast.getMastDistance());
    SmartDashboard.putNumber("PID mast", hangarMast.getMastEncoder());
    SmartDashboard.putNumber("PID mast distance", hangarMast.getMastDistance());
    SmartDashboard.putNumber("PID kPmast", Constants.Hanger.HangarPIDConstants.kParm);
    SmartDashboard.putNumber("PID mast setpoint", mastDISTANCE1);
    SmartDashboard.putNumber("PID mast ERROR", mastERROR);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangarMast.mastStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean isMastMargin = mastERROR < mastMARGIN;
    SmartDashboard.putBoolean("mast PID finished", isMastMargin);
    return false;
  }
}
