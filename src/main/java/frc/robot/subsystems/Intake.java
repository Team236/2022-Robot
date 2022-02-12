// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private DoubleSolenoid intakeSolenoid;
  private Counter ballCounter;
  private boolean isCounterUnplugged = false;

  /** Creates a new Intake. */
  public Intake() {

    intakeMotor = new CANSparkMax(Constants.IntakeConstants.ID_MOTOR, MotorType.kBrushless);
    intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
  }

  public void setSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  public void extend() {
    intakeSolenoid.set(Value.kForward);
  }

  public boolean isExtended() {
    if (intakeSolenoid.get() == Value.kForward) {
      return true;
    } else {
      return false;
    }
    // return intakeSolenoid.get() == Value.kForward;
  }


  public void retract() {
    intakeSolenoid.set(Value.kReverse);
  }

  public int getBallCount() {
    if (isCounterUnplugged) {
      return 0;
    } else {
      return ballCounter.get();
    }
  }

  public void resetCounter() {
    ballCounter.reset();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
