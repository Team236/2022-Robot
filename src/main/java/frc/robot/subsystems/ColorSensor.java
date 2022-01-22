// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;

import org.ejml.dense.row.misc.DeterminantFromMinor_DDRM;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase {

  private ColorSensorV3 colorSensor;
  private ColorMatch colorMatcher;
  // private DriverStation.Alliance allianceColor;
  private String colorString;
  
  /** Creates a new ColorSensor. */
  public ColorSensor() {

    I2C.Port i2cPort = I2C.Port.kMXP;
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatcher = new ColorMatch();
    
    // blueBall = Constants.ColorSensorConstants.BLUE;
    // redBall = RED;
  }

  // public String getAllianceColor() {
  //   allianceColor = DriverStation.getAlliance();
  //   return allianceColor;
  // }

  public void getColor() {
    
    Color detectedColor = colorSensor.getColor();
    double IR = colorSensor.getIR();
    RawColor rawColor = colorSensor.getRawColor();
    String myRawColor = rawColor.toString();

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("IR", IR);
    SmartDashboard.putString("raw color", myRawColor);

    ColorMatchResult closestColor = colorMatcher.matchClosestColor(detectedColor);
    String myClosestColor = closestColor.toString();
    SmartDashboard.putString("closest color", myClosestColor);
    
  }

  // public void checkBallColor() {
  //   // if ((colorString == "Blue") & (DriverStation.Alliance == blue)) {
  //   //   SmartDashboard.putBoolean("correct ball", true);
  //   // }
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
