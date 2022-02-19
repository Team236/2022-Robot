// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.commands.DriveWithPID;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.lang.Math;



//kX is a variable that deterimines the coefficent of force we would need to use to cause the robot to swivel. 
public class AnglewithLL extends CommandBase {
private double kX = 0.02;
private Drive drive;


public AnglewithLL(Drive drive2) {
  drive = drive2;
}

/** Creates a new DrivewithLimelight. **/
  public void drivewithLimelight(/*Drive drive*/) {

   // this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(6);

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override 
  public void execute() {
  //aims with limelight 
    //NetworkTable table = NetworkTableInstance.getDefault().getTable("limelght");
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    //tx is the angle on the x axis of the target
    double errorX = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    //confirms a target 
    if(tv==1){

      //double x = (errorX-160)/320;
        //Establishes a minimum error in the x axis 
        if(Math.abs(errorX)>2){
          System.out.println("errorX " +errorX);
         double steeringAdjust = kX * errorX;
         System.out.println("steering adjust " + steeringAdjust);
         drive.setLeftSpeed(steeringAdjust);
         drive.setRightSpeed(-steeringAdjust); 
      }
     

    }
    
  
  else{
    SmartDashboard.putNumber("No Target", tv);
  }
  
  //

      //double a2 = (Math.atan(h2-h1)/dx)-a1 ;

      
      //
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
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}


