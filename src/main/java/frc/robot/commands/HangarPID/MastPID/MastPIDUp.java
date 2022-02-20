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

  private Hangar hangar;
  private double mastDISTANCE;
  private double mastMARGIN;
  private double mastERROR;
  
  /** Creates a new HangerMastPID. */
  public MastPIDUp(Hangar hangar, double mastDISTANCE, double mastMARGIN) {

    this.hangar = hangar;
    addRequirements(hangar);
    // Use addRequirements() here to declare subsystem dependencies.

    this.mastDISTANCE = mastDISTANCE;
    this.mastMARGIN = mastMARGIN;
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    hangar.resetEncoders();
    hangar.setMastkP(Constants.Hanger.HangarPIDConstants.kPmast);
    hangar.setMastkI(Constants.Hanger.HangarPIDConstants.kImast);
    hangar.setMastkD(Constants.Hanger.HangarPIDConstants.kDmast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hangar.setMastOutputRange();
    hangar.setMastSetPoint(mastDISTANCE);

    mastERROR = Math.abs(mastDISTANCE - hangar.getMastDistance());
    
    SmartDashboard.putNumber("PID mast revs", hangar.getMastEncoder());
    SmartDashboard.putNumber("PID mast distance", hangar.getMastDistance());
    SmartDashboard.putNumber("PID kPmast", Constants.Hanger.HangarPIDConstants.kParm);
    SmartDashboard.putNumber("PID mast setpoint", mastDISTANCE);
    SmartDashboard.putNumber("PID mast ERROR", mastERROR);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hangar.mastStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //boolean isMastMargin = mastERROR < mastMARGIN;
    //SmartDashboard.putBoolean("mast PID finished", isMastMargin);
    return false;
  }
}
