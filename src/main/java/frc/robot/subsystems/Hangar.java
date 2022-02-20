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

  public double getMastDistance() {
    return getMastEncoder() * Constants.Hanger.HangarPIDConstants.mastREV_TO_IN;
  } 

  public double getArmDistance() {
    return getArmEncoder() * Constants.Hanger.HangarPIDConstants.armREV_TO_IN;

  }
  //different distances for each instance of PID
  public void setArmSetPoint1() {
   armPID.setReference((Constants.Hanger.HangarPIDConstants.armDISTANCE1 * Constants.Hanger.HangarPIDConstants.armIN_TO_REV), ControlType.kPosition);
  }
  public void setArmSetPoint2() {
    armPID.setReference((Constants.Hanger.HangarPIDConstants.armDISTANCE2 * Constants.Hanger.HangarPIDConstants.armIN_TO_REV ), ControlType.kPosition);
  }
  public void setArmSetPoint3() {
    armPID.setReference((Constants.Hanger.HangarPIDConstants.armDISTANCE3 * Constants.Hanger.HangarPIDConstants.armIN_TO_REV), ControlType.kPosition);
  }
 
  public void setMastSetPoint1() {
    mastPID.setReference((Constants.Hanger.HangarPIDConstants.mastDISTANCE1 * Constants.Hanger.HangarPIDConstants.mastIN_TO_REV), ControlType.kPosition); 
  } 

  //methods for setting hangar-specific PID values
  public void setMastkP(double kPmast) {
    mastPID.setP(kPmast);
  }

  public void setArmkP(double kParm) {
    armPID.setP(kParm);
  }

  public void setMastkI(double kImast) {
    mastPID.setI(kImast);
  }

  public void setArmkI(double kIarm) {
    armPID.setI(kIarm);
  }

  public void setMastkD(double kDmast) {
    mastPID.setD(kDmast);
  }

  public void setArmkD(double kDarm) {
    armPID.setD(kDarm);
  }

  public void setMastkF(double kFmast) {
    mastPID.setFF(kFmast);
  }

  public void setArmkF(double kFarm) {
    armPID.setFF(kFarm);
  }

  public void setArmOutputRange() {
    mastPID.setOutputRange(Constants.Hanger.HangarPIDConstants.hangarMIN_OUTPUT, Constants.Hanger.HangarPIDConstants.hangarMAX_OUTPUT);
    armPID.setOutputRange(Constants.Hanger.HangarPIDConstants.hangarMIN_OUTPUT, Constants.Hanger.HangarPIDConstants.hangarMAX_OUTPUT);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
