// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Drive;

// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Drive;

// public class TrackBall extends CommandBase {

//   private double kX = 0.01;
//   private Drive drive;
//   private String allianceColor;

//   /** Creates a new TrackBall. */
//   public TrackBall(Drive drive) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.drive = drive;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     allianceColor = DriverStation.getAlliance().toString();

//     if (allianceColor == "Red") {
//       NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("pipeline").setNumber(0);
//     } else if (allianceColor == "Blue") {
//       NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("pipeline").setNumber(1);
//     }

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     double tv = NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("tv").getDouble(0);
//     double errorX = NetworkTableInstance.getDefault().getTable("limelight-intake").getEntry("tx").getDouble(0);
//     if(tv==1){
//       if(Math.abs(errorX)>2){
//         double steeringAdjust = kX * errorX;
//         drive.setLeftSpeed(steeringAdjust);
//         drive.setRightSpeed(-steeringAdjust); 
//       }
//     } else {
//     SmartDashboard.putNumber("No Ball Target", tv);
//     }

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     drive.stop();
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
