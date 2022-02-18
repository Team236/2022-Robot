// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.ColorMatch;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorSensorV3.RawColor;


// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.I2C;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.ColorSensorConstants;

// public class ColorSensor extends SubsystemBase {

//   private ColorSensorV3 colorSensor;
//   private ColorMatch colorMatcher;
//   private String colorString;
  
//   /** Creates a new ColorSensor. */
//   public ColorSensor() {

//     I2C.Port onGyro = I2C.Port.kMXP;
//     I2C.Port onRio = I2C.Port.kOnboard;
//     colorSensor = new ColorSensorV3(onGyro);
//     colorMatcher = new ColorMatch();
    
//   }

//   public Alliance getAllianceColor() {

//     return DriverStation.getAlliance();
//   }

//   public String publishAllianceColor(){
//     String myAlliance = "Invalid";

//     if (getAllianceColor() == Alliance.Red) {
//       myAlliance = "Red";
//     } else if (getAllianceColor() == Alliance.Blue) {
//       myAlliance = "Blue";
//     } else if (getAllianceColor() == Alliance.Invalid) {
//       myAlliance = "Invalid";
//     }

//     SmartDashboard.putString("myAlliance", myAlliance);

//     return myAlliance;
//   }

//   public void getColor() {
    
//     Color detectedColor = colorSensor.getColor();
//     // RawColor rawColor = colorSensor.getRawColor();
//     // String myRawColor = rawColor.toString();
//     // String myGetColor = detectedColor.toString();

//     // SmartDashboard.putNumber("get red", colorSensor.getRed());
//     // SmartDashboard.putString("getColor string", myGetColor);

//     SmartDashboard.putNumber("Red", detectedColor.red);
//     SmartDashboard.putNumber("Green", detectedColor.green);
//     SmartDashboard.putNumber("Blue", detectedColor.blue);
//     // SmartDashboard.putString("raw color", myRawColor);
    
//   }

//   public double getRed() {
//     Double redDouble = colorSensor.getColor().red;
//     return redDouble;
//   }

//   public double getBlue() {
//     Double blueDouble = colorSensor.getColor().blue;
//     return blueDouble;
//   }

//   public boolean isRedGreater() {
//     return (getRed() > getBlue());
//   }

//   public boolean isBlueGreater() {
//     return (getBlue() > getRed());
//   }

//   public void isBallMine() {
//     if ((publishAllianceColor() == "Red") && (getRed() > getBlue())) {
//       SmartDashboard.putBoolean("isBallMine", true);
//     } else if ((publishAllianceColor() == "Blue") && (getBlue() > getRed())) {
//       SmartDashboard.putBoolean("isBallMine", true);
//     } else SmartDashboard.putBoolean("isBallMine", false);
//   }

//   public int getDistance() {
//     SmartDashboard.putNumber("color sensor distance", colorSensor.getProximity());
//     return colorSensor.getProximity();
//   }

//   public boolean isBallInSpoon() {
//     return (getDistance() > ColorSensorConstants.DIST);
//   }

//   public void whatColor() {
//     if (getDistance() < ColorSensorConstants.DIST) {
//       SmartDashboard.putBoolean("blue ball in spoon", false);
//       SmartDashboard.putBoolean("red ball in spoon", false);
//     } else if ((getDistance() > ColorSensorConstants.DIST) && (isRedGreater())) {
//       SmartDashboard.putBoolean("blue ball in spoon", false);
//       SmartDashboard.putBoolean("red ball in spoon", true);
//     } else if ((getDistance() > ColorSensorConstants.DIST) && (isBlueGreater())) {
//       SmartDashboard.putBoolean("blue ball in spoon", true);
//       SmartDashboard.putBoolean("red ball in spoon", false);
//     }
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     whatColor();
//   }
// }
