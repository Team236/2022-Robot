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
import frc.robot.subsystems.Mast;

public class MastPID extends CommandBase {

  private Mast mast;
  private double mastDistance;
  private double mastMargin;
  private double mastError;
  
  /** Creates a new HangerMastPID. */
  public MastPID(Mast mast, double mastDistance, double mastMargin) {

    this.mast = mast;
    addRequirements(mast);
    // Use addRequirements() here to declare subsystem dependencies.
    this.mastDistance = mastDistance;
    this.mastMargin = mastMargin;
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {
    mast.setMastkP(ClimberConstants.kPmast);
    mast.setMastkI(ClimberConstants.kImast);
    mast.setMastkD(ClimberConstants.kDmast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    mast.setMastOutputRange();
    mast.setMastSetPoint(mastDistance);

    mastError = Math.abs(mastDistance - mast.getMastEncoder());
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mast.mastStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return mast.mastLimitTriggered();

    if ((mastDistance > 0) && mast.isMExtendLimit()) {
      return true;
    } else if ((mastDistance < 0) && mast.isMReturnLimit()) {
      mast.resetMastEncoder();
      return true;
    } else {
      return false;
    }
  }
}
