// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Shooter extends SubsystemBase {

//   private CANSparkMax bottomRoller, topRoller;
//   private SparkMaxPIDController botPidController, topPidController;
//   private RelativeEncoder botEncoder, topEncoder;

//   /** Creates a new Shooter. */
//   public Shooter() {

//     bottomRoller = new CANSparkMax(39, MotorType.kBrushless);
//     topRoller = new CANSparkMax(7, MotorType.kBrushless);

//     bottomRoller.restoreFactoryDefaults();
//     topRoller.restoreFactoryDefaults();

//     botPidController = bottomRoller.getPIDController();
//     topPidController = topRoller.getPIDController();

//     botEncoder = bottomRoller.getEncoder();
//     topEncoder = topRoller.getEncoder();

//   }

//   public void setBotSetPoint(double botSpeed) {
//     botPidController.setReference(botSpeed, ControlType.kVelocity);
//   }

//   public void setTopSetPoint(double topSpeed) {
//     topPidController.setReference(topSpeed, ControlType.kVelocity);
//   }

//   public void setP(double kP) {
//     botPidController.setP(kP);
//     topPidController.setP(kP);
//   }

//   public void setI(double kI) {
//     botPidController.setI(kI);
//     topPidController.setI(kI);
//   }

//   public void setD(double kD) {
//     botPidController.setD(kD);
//     topPidController.setD(kD);
//   }

//   public void setFF(double kF) {
//     botPidController.setFF(kF);
//     topPidController.setFF(kF);
//   }

//   public void setOutputRange() {
//     topPidController.setOutputRange(0.0, 5000.0);
//     botPidController.setOutputRange(0.0, 5000.0);
//   }

//   public void resetEncoders() {
//     botEncoder.setPosition(0);
//     topEncoder.setPosition(0);
//   }

//   public double getBotEncoder() {
//     return botEncoder.getPosition();
//   }

//   public double getTopEncoder() {
//     return topEncoder.getPosition();
//   }

//   public double getBotVelocity() {
//     return topEncoder.getVelocity();
//   }

//   public double getTopVelocity() {
//     return botEncoder.getVelocity();
//   }

//   public void setBotSpeedRaw(double botSpeed) {
//     bottomRoller.set(botSpeed);
//   }

//   public void setTopSpeedRaw(double botSpeed) {
//     topRoller.set(botSpeed);
//   }

//   public void stop() {
//     setBotSpeedRaw(0);
//     setTopSpeedRaw(0);
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     SmartDashboard.putNumber("bot shoot velocity", getBotVelocity());
//     SmartDashboard.putNumber("top shoot velocity", getTopVelocity());
//   }
// }
