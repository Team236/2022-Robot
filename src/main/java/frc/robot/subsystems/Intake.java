// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/*public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private DoubleSolenoid intakeSolenoid;

  /** Creates a new Intake. */
  /*public Intake() {

    intakeMotor = new CANSparkMax(Constants.IntakeConstants.ID_MOTOR, MotorType.kBrushless);

    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  } 

  /*public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public void extend() {
    intakeSolenoid.set(Value.kForward);
  }

  public boolean isExtended() {
    return intakeSolenoid.get() == Value.kForward;
  }

  public void retract() {
    intakeSolenoid.set(Value.kReverse);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} */
