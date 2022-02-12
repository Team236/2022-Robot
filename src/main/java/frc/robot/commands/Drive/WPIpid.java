// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class WPIpid extends CommandBase {

  private Drive drive;
  private PIDController m_pidController;
  private CANSparkMax leftFront, rightFront;
  private double pidLOut, pidROut;
  private boolean isReachedSetpoint;

  /** Creates a new NewPID. */
  public WPIpid(Drive pidDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = pidDrive;
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    drive.resetEncoders();
    m_pidController = new PIDController(0.0005, 0, 0);
    m_pidController.setSetpoint(5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidLOut = m_pidController.calculate(drive.getLeftEncoder());
    pidROut = m_pidController.calculate(drive.getLeftSpeed());
    leftFront.set(drive.pidLOut());
    rightFront.set(drive.pidROut());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
