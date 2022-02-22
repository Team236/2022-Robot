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
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {

  private CANSparkMax armMotor, mastMotor;
  private RelativeEncoder armEncoder, mastEncoder;
  private SparkMaxPIDController armPID, mastPID;

  /** Creates a new Climber. */
  public Climber() {

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
   // mastMotor.restoreFactoryDefaults();
    mastMotor.set(ClimberConstants.MAST_EX_SPEED);
  }

  public void armOut() {
    //armMotor.restoreFactoryDefaults();
    armMotor.set(ClimberConstants.ARM_EX_SPEED);
  }

  public void mastIn() {
   // mastMotor.setInverted(true);
    mastMotor.set(-ClimberConstants.MAST_RE_SPEED);
  }

  public void armIn() {
    //armMotor.setInverted(true);
    armMotor.set(-ClimberConstants.ARM_RE_SPEED);
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
    return getMastEncoder() * ClimberConstants.mastREV_TO_IN;
  } 

  public double getArmDistance() {
    return getArmEncoder() * ClimberConstants.armREV_TO_IN;
  }

  //different distances for each instance of PID
  //armDISTANCE is in INCHES not revs
  public void setArmSetPoint(double armDISTANCE) {
   armPID.setReference((armDISTANCE * ClimberConstants.armIN_TO_REV), ControlType.kPosition);
  }
 
  public void setMastSetPoint(double mastDISTANCE) {
    mastPID.setReference((mastDISTANCE * ClimberConstants.armIN_TO_REV), ControlType.kPosition); 
  } 

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
    armPID.setOutputRange(ClimberConstants.climberMIN_OUTPUT, ClimberConstants.climberMAX_OUTPUT);
  }
  public void setMastOutputRange() {
    mastPID.setOutputRange(ClimberConstants.climberMIN_OUTPUT, ClimberConstants.climberMAX_OUTPUT);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}
