// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.Hanger.ClimberConstants;
import frc.robot.Constants.Hanger.HangarPIDConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hangar extends SubsystemBase {

  private CANSparkMax armMotor, mastMotor;
  private RelativeEncoder armEncoder, mastEncoder;
  private SparkMaxPIDController armPID, mastPID;

  /** Creates a new Hangar. */
  public Hangar() {

    armMotor = new CANSparkMax(Constants.MotorControllers.ID_ARM, MotorType.kBrushless);
    mastMotor = new CANSparkMax(Constants.MotorControllers.ID_MAST, MotorType.kBrushless);

    mastMotor.setInverted(false);
    armMotor.setInverted(false);

    mastPID = mastMotor.getPIDController();
    armPID = armMotor.getPIDController();

    mastEncoder = mastMotor.getEncoder();
    armEncoder = armMotor.getEncoder();


  }

  public void mastOut() {
    mastMotor.restoreFactoryDefaults();
    mastMotor.set(Constants.Hanger.ClimberConstants.MAST_EX_SPEED);

  }

  public void armOut() {
    armMotor.restoreFactoryDefaults();
    armMotor.set(Constants.Hanger.ClimberConstants.ARM_EX_SPEED);

  }

  public void mastIn() {
    mastMotor.setInverted(true);
    mastMotor.set(Constants.Hanger.ClimberConstants.MAST_RE_SPEED);

  }

  public void armIn() {
    armMotor.setInverted(true);
    armMotor.set(Constants.Hanger.ClimberConstants.ARM_RE_SPEED);

  }

  public void mastStop() {
    mastMotor.set(0);

  }

  public void armStop() {
    mastMotor.set(0);

  }

  public void resetEncoders() {
    mastEncoder.setPosition(0);
    armEncoder.setPosition(0);
  }

  //returns encoder position in REVOLUTIONS 
  public double getMastEncoder() {
     return mastEncoder.getPosition();
  }

  public double getArmEncoder() {
    return armEncoder.getPosition();
  }

 /* public double getMastDistance() {
    return getMastEncoder() * mastREV_TO_IN;
  } */

  public double getArmDistance() {
    return getArmEncoder() * Constants.Hanger.HangarPIDConstants.armREV_TO_IN;

  }
  //different distances for each instance of PID
  public void setArmSetPoint1() {
   armPID.setReference((Constants.Hanger.HangarPIDConstants.armDISTANCE1 * Constants.Hanger.HangarPIDConstants.armIN_TO_REV), ControlType.kPosition);
  }

  /* public void setMastSetPoint1() {
    mastPID.setReference((Constants.Hanger.HangarPIDConstants.mastDISTANCE1 * Constants.Hanger.HangarPIDCOnstants.mastIN_TO_REV), ControlType.kPostion); 
  } */

  //methods for setting hangar-specific PID values
  public void setMastkP() {
    mastPID.setP(Constants.Hanger.HangarPIDConstants.kPmast);
  }

  public void setArmkP() {
    armPID.setP(Constants.Hanger.HangarPIDConstants.kParm);
  }

  public void setMastkI() {
    mastPID.setI(Constants.Hanger.HangarPIDConstants.kImast);
  }

  public void setArmkI() {
    armPID.setI(Constants.Hanger.HangarPIDConstants.kIarm);
  }

  public void setMastkD() {
    mastPID.setD(Constants.Hanger.HangarPIDConstants.kDmast);
  }

  public void setArmkD() {
    armPID.setD(Constants.Hanger.HangarPIDConstants.kDarm);
  }

  public void setMastkF() {
    mastPID.setFF(Constants.Hanger.HangarPIDConstants.kFmast);
  }

  public void setArmkF() {
    armPID.setFF(Constants.Hanger.HangarPIDConstants.kFarm);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
