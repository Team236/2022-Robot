// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intaketwo extends SubsystemBase {
  /** Creates a new Intake. */
 // private Spark intakeMotor;
 // private Spark raiseLowerMotor;
//  private DigitalInput upperLimit, lowerLimit;

  // private Counter ballCounter;
  // private boolean isCounterUnplugged = false;
  // private boolean limitsUnplugged = false;
  /** Creates a new Intaketwo. */
  public Intaketwo() {
    DoubleSolenoid intakeSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.SOL_FWD, Constants.IntakeConstants.SOL_REV);
    DoubleSolenoid intakeScoreSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.SCORE_SOL_FWD, Constants.IntakeConstants.SCORE_SOL_REV);
    // Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    // Compressor phCompressor = new Compressor(1, PneumaticsModuleType.CTREPCM);



// boolean enabled = pcmCompressor.enabled();
// boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
// double current = pcmCompressor.getCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
