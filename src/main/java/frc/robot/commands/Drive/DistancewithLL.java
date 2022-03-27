// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;



//kX is a variable that deterimines the coefficent of force we would need to use to cause the robot to swivel. 
public class DistancewithLL extends CommandBase {
private double kY = 0.00785; //0.00725;
private Drive drive;
private double h1 = 13.5;
private double h2 = 105.6;
private double a1 = 0.8340573450259819; //47.78 degrees or 17.98
private double d = 78 * 65 / 70;
// safety is 166
// first number is the inches value that you put in
// 1.0769 factor from 
// limelight is positioned at 40 degrees from horizontal


public DistancewithLL(Drive drive2) {
  drive = drive2;
}

/** Creates a new DrivewithLimelight. **/
  public void distancewithLL(/*Drive drive*/) {

    // this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ledMode").setNumber(3);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
  //aims with limelight 
    //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelght");
    NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ledMode").setNumber(3);
    double tv = NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("tv").getDouble(0);
    //tx is the angle on the x axis of the target
    double disY= NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ty").getDouble(0);
    //confirms a target 
    if(tv==1){
      
       // ty is the angle on the y axis of the target
        //double a2 = ((disY-120)/240)*Math.PI/180;
          double a2 = disY*Math.PI/180;
          // System.out.println("a2 "+ a2);
          double dx = (h2-h1) / Math.tan(a1+a2);
          // System.out.println("dx " + dx);
          double errorY = d - dx;
          // System.out.println("errorY "+ errorY);
          double distanceAdjust = kY * errorY;
         //System.out.println("distanceAdjust " + distanceAdjust);
     // if (distanceaAdjust > 0) {
        drive.setLeftSpeed(distanceAdjust);
        drive.setRightSpeed(distanceAdjust);

    } else{
    // SmartDashboard.putNumber("No Target", tv);
    }
      //double a2 = (Math.atan(h2-h1)/dx)-a1 ;
      // }
      //here I intend to use the steering adjust to 
      // if (Errorx < .5) {
      //   pipelineEntry.setNumber(2);
      // }
      // if (Error < .2) {
      //   pipelineEntry.setNumber(3);
      // }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    NetworkTableInstance.getDefault().getTable("limelight-shooter").getEntry("ledMode").setNumber(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
