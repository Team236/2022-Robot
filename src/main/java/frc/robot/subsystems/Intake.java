// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.AnalogInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;


import static frc.robot.Constants.IntakeConstants.*;


public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private Spark intakeMotor;
  private Spark raiseLowerMotor;
  private DigitalInput upperLimit, lowerLimit;

  private Counter ballCounter;
  private boolean isCounterUnplugged = false;
  private boolean limitsUnplugged = false;

//Probably need to change for this year's code
  public Intake() {
    intakeMotor = new Spark(ID_MOTOR);
    raiseLowerMotor = new Spark(ID_POSITION_MOTOR);
    raiseLowerMotor.setInverted(true);

    // raiseLowerMotor.getForwardLimitSwitch();
    // raiseLowerMotor.getReverseLimitSwitch();

    // Limit switches
    // try {
    //   upperLimit = new DigitalInput(DIO_UPPER_LIMIT);
    //   lowerLimit = new DigitalInput(DIO_LOWER_LIMIT);
    // } catch (Exception e) {
    //   limitsUnplugged = true;
    // }

    // Ball counter
    //try {
    //   this.ballCounter = new Counter();
    //   this.ballCounter.setUpSource(DIO_INTAKE_COUNTER);
    //   this.ballCounter.setDownSource(Constants.DIO_SHOOT_COUNTER);
    // } catch (Exception e) {
    //   isCounterUnplugged = true;
    // }
  
  }

  public void setSpeed(double speed) {
    // if (true) {
     // intakeMotor.set(ControlMode.PercentOutput, -speed);

    // }
    // !getLowerLimit() || speed > 0
  }

  public void stop() {
    setSpeed(0);
  }

  /**
   * Reads ball counter (intake & shooter counters)
   * 
   * @return number of balls in robot
   */
  public int getBallCount() {
    if (isCounterUnplugged) {
      return 0;
    } else {
      return ballCounter.get();
    }
  }

  /**
   * Resets intake counter to 0
   */
  public void resetCounter() {
    ballCounter.reset();
  }

  public boolean getUpperLimit() {
    if (limitsUnplugged) {
      return false;
    } else {
      return !upperLimit.get();
    }
  }

  public boolean getLowerLimit() {
    // return raiseLowerMotor.getSensorCollection().isRevLimitSwitchClosed();
     if (limitsUnplugged) {
      return false;
    } else {
      return !lowerLimit.get();
    } 
  }

  /**
   * Sets speed of motor that positions intake up/down
   * 
   * @param speed
   */
  public void setPositionSpeed(double speed) {
  //   if (speed > 0 && getUpperLimit()) {
  //     stopPositionMotor();
  //   } else if (speed < 0 && getLowerLimit()) {
  //     stopPositionMotor();
  //   } else {
  //     raiseLowerMotor.set(ControlMode.PercentOutput, speed);
  //   }

  }

  public void raise() {

  }

 public void lower() {
  //   double rot = -stick.getX();
  //   	double speed = -stick.getY();
  //      ArmsDouble= new DoubleSolenoid(null, 1, 2);
  //      ArmsDouble= new DoubleSolenoid(1, null, 1, 2);
  //       if(stick.getRawButton(4)){
  //       	ArmsDouble.set(DoubleSolenid.Value.kForward);
        	
  //       }
  //       else if(stick.getRawButton(5)){
  //       	ArmsDouble.set(DoubleSolenoid.Value.kReverse);
    
  //       }
  }

  

  public void stopPositionMotor() {
    setPositionSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("int down lim", getLowerLimit());
  }
}
