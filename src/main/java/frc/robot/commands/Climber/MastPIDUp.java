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

public class MastPIDUp extends CommandBase {

  private Climber climber;
  private double mastDISTANCE;
  private double mastMARGIN;
  private double mastERROR;
  
  /** Creates a new HangerMastPID. */
  public MastPIDUp(Climber climber, double mastDISTANCE, double mastMARGIN) {

    this.climber = climber;
    addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.

    this.mastDISTANCE = mastDISTANCE;
    this.mastMARGIN = mastMARGIN;
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    climber.resetEncoders();
    climber.setMastkP(ClimberConstants.kPmast);
    climber.setMastkI(ClimberConstants.kImast);
    climber.setMastkD(ClimberConstants.kDmast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climber.setMastOutputRange();
    climber.setMastSetPoint(mastDISTANCE);

    mastERROR = Math.abs(mastDISTANCE - climber.getMastDistance());
    
    SmartDashboard.putNumber("PID mast revs", climber.getMastEncoder());
    SmartDashboard.putNumber("PID mast distance", climber.getMastDistance());
    SmartDashboard.putNumber("PID kPmast", ClimberConstants.kParm);
    SmartDashboard.putNumber("PID mast setpoint", mastDISTANCE);
    SmartDashboard.putNumber("PID mast ERROR", mastERROR);
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.mastStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //boolean isMastMargin = mastERROR < mastMARGIN;
    //SmartDashboard.putBoolean("mast PID finished", isMastMargin);
    return false;
  }
}
