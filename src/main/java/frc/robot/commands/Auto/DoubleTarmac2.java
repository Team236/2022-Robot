// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.AngleAndDistLL;
import frc.robot.commands.Drive.WPI_PID;
import frc.robot.commands.Drive.WPI_Turn_PID;
import frc.robot.commands.Hood.HoodExtend;
import frc.robot.commands.Hood.HoodRetract;
import frc.robot.commands.Intake.IntakeExtend;
import frc.robot.commands.Intake.IntakeForward;
import frc.robot.commands.Intake.SetIntakeSpeed;
import frc.robot.commands.Shooter.Shoot;
import frc.robot.commands.Shooter.SpoonAndShoot;
import frc.robot.commands.Spoon.SpoonCmdGroup;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LoadingSpoon;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleTarmac2 extends SequentialCommandGroup {
  /** Creates a new DoubleTarmacWithLL. */
  public DoubleTarmac2(Drive drive, Intake intake, LoadingSpoon loadingSpoon, Shooter shooter, Hood hood) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      parallel(
        new Shoot(shooter, ShooterConstants.TARMAC_BOT, ShooterConstants.TARMAC_TOP),
        sequence(
          new IntakeExtend(intake, true).withTimeout(1),
          parallel(
            new IntakeForward(intake, IntakeConstants.FORWARD_SPEED),
            new WPI_PID(drive, DriveConstants.TARMAC_TO_BALL),
            new HoodRetract(hood)
          ).withTimeout(2),
          new WPI_Turn_PID(drive, -DriveConstants.TURN_18).withTimeout(0.5),
          new WPI_PID(drive, -DriveConstants.BALL_TO_LINE).withTimeout(2),
          new SpoonCmdGroup(loadingSpoon).withTimeout(1),
          new SetIntakeSpeed(intake, IntakeConstants.FORWARD_SPEED).withTimeout(2),
          new SpoonCmdGroup(loadingSpoon).withTimeout(1)
        )
      )
    );
  }
}
