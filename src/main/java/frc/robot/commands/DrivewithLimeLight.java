package frc.robot.commands;


import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;


//kX is a variable that deterimines the coefficent of force we would need to use to cause the robot to swivel. 
public class DrivewithLimeLight extends CommandBase {
private double kX = 0.01;
private double kY = 0.03;
private double Ks = 0.02;
private Drive drive;
private double defaultValue = 999;
private double h1 = 13.5;
private double h2 = 105.6;
private double a1 = 0.8340573450259819; //47.78 d egrees or 17.98
private double d = 85;

public DrivewithLimeLight(Drive drive2) {
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
    System.out.println(errorX);
    //confirms a target 
    if(tv==1){

      //double x = (errorX-160)/320;
        //Establishes a minimum error in the x axis 
        if(Math.abs(errorX)>1){
         double steeringAdjust = kX * errorX;
        //  System.out.println("steering adjust " + steeringAdjust);
         drive.setLeftSpeed(steeringAdjust);
         drive.setRightSpeed(-steeringAdjust); 
      }
      
    //     else{
    //     //ty is the angle on the y axis of the target
    //       double disY = table.getEntry("ty").getDouble(defaultValue);
    //  // double a2 = ((disY-120)/240)*Math.PI/180;
    //       double a2 = disY*Math.PI/180;
    //       double dx = (h2-h1) / Math.tan(a1+a2);
    //       double errorY = d - dx;
    //       double distanceaAdjust = kY * errorY;
    //  // if (distanceaAdjust > 0) {
    //     drive.setLeftSpeed(distanceaAdjust);
    //     drive.setRightSpeed(distanceaAdjust);
    //  // }
    //   //    else{
    //   //    drive.setLeftSpeed(distanceaAdjust);
    //   //    drive.setRightSpeed(distanceaAdjust);
    //   // }

    // //}
    // }
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