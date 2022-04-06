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
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Intake.NewIntakeForward;
import frc.robot.commands.Shooter.FeedAndShoot;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleTarmac2 extends SequentialCommandGroup {
  /** Creates a new DoubleTarmacWithLL. */
  public DoubleTarmac2(Drive drive, Intake intake, Shooter shooter, Hood hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeExtend(intake).withTimeout(0.5),
      parallel(
        new NewIntakeForward(intake),
        new WPI_PID(drive, DriveConstants.TARMAC_TO_BALL),
        new HoodRetract(hood)
      ).withTimeout(1.5),
      new WPI_Turn_PID(drive, -22).withTimeout(1),
      new FeedAndShoot(intake, shooter, hood, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP).withTimeout(4),
      new IntakeExtend(intake).withTimeout(0.5)
    );
  }
}
