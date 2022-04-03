// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Drive.WPI_Turn_PID;
import frc.robot.commands.Hood.HoodRetract;
import frc.robot.commands.Intake.AutoIntake;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Shooter.FeedAndShoot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleTarmac1 extends SequentialCommandGroup {
  /** Creates a new DoubleTarmacShot. */
  public DoubleTarmac1(Drive drive, Shooter shooter, Hood hood, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      sequence(
        // extend intake
        new IntakeExtend(intake).withTimeout(0.5),
        parallel(
          // drive to ball while intaking and extending hood
          new AutoIntake(intake),
          new WPI_PID(drive, DriveConstants.TARMAC_TO_BALL_SHORT),
          new HoodRetract(hood)
        ).withTimeout(1.5),
        // shoot two balls using feed wheels
        new FeedAndShoot(intake, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP).withTimeout(2.2),
        // extend intake for teleop        
        parallel(
          new WPI_Turn_PID(drive, 98),
          new IntakeExtend(intake)
        ).withTimeout(1)      
      )
    );
  }
}
