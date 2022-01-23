// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.RawColor;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ColorSensor extends SubsystemBase {

  private ColorSensorV3 colorSensor;
  private ColorMatch colorMatcher;
  private String colorString;
  
  /** Creates a new ColorSensor. */
  public ColorSensor() {

    I2C.Port i2cPort = I2C.Port.kMXP;
    colorSensor = new ColorSensorV3(i2cPort);
    colorMatcher = new ColorMatch();
    
  }

  public Alliance getAllianceColor() {

    return DriverStation.getAlliance();
  }

  public String publishAllianceColor(){
    String myAlliance = "Invalid";

    if (getAllianceColor() == Alliance.Red) {
      myAlliance = "Red";
    } else if (getAllianceColor() == Alliance.Blue) {
      myAlliance = "Blue";
    } else if (getAllianceColor() == Alliance.Invalid) {
      myAlliance = "Invalid";
    }

    SmartDashboard.putString("myAlliance", myAlliance);

    return myAlliance;
  }

  public void getColor() {
    
    Color detectedColor = colorSensor.getColor();
    // RawColor rawColor = colorSensor.getRawColor();
    // String myRawColor = rawColor.toString();
    // String myGetColor = detectedColor.toString();

    // SmartDashboard.putNumber("get red", colorSensor.getRed());
    // SmartDashboard.putString("getColor string", myGetColor);

    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    // SmartDashboard.putString("raw color", myRawColor);

    // ColorMatchResult closestColor = colorMatcher.matchClosestColor(detectedColor);
    // SmartDashboard.putString("closest color", closestColor.color.toString());
    // String myClosestColor = closestColor.toString();
    // SmartDashboard.putString("closest color string", myClosestColor);
    // colorMatcher.matchClosestColor(detectedColor);
    
  }

  public double getRed() {
    Double redDouble = colorSensor.getColor().red;
    return redDouble;
  }

  public double getBlue() {
    Double blueDouble = colorSensor.getColor().blue;
    return blueDouble;
  }

  public void isBallMine() {
    if ((publishAllianceColor() == "Red") && (getRed() > getBlue())) {
      SmartDashboard.putBoolean("isBallMine", true);
    } else if ((publishAllianceColor() == "Blue") && (getBlue() > getRed())) {
      SmartDashboard.putBoolean("isBallMine", true);
    } else SmartDashboard.putBoolean("isBallMine", false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
