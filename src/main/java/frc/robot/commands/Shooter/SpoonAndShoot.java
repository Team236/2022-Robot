// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Spoon.SpoonExtend;
import frc.robot.commands.Spoon.SpoonRetract;
import frc.robot.subsystems.LoadingSpoon;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpoonAndShoot extends SequentialCommandGroup {
  /** Creates a new SpoonAndShoot. */
  public SpoonAndShoot(LoadingSpoon loadingSpoon, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // runs shooter in parallel with wait/spoonExtend/spoonRetract sequence
    addCommands(
      parallel(
        new Shoot(shooter, ShooterConstants.HIGH_HUB_LARGE, ShooterConstants.HIGH_HUB_SMALL).withTimeout(6), 
        sequence(
          new WaitCommand(2), 
          new SpoonExtend(loadingSpoon).withTimeout(2), 
          new SpoonRetract(loadingSpoon).withTimeout(1)
          )
        )
      );

    /*
    super(sequence(
      new Shoot(shooter, ShooterConstants.HIGH_HUB_LARGE, ShooterConstants.HIGH_HUB_SMALL),
      new WaitCommand(0.1),
      new SpoonExtend(loadingSpoon),
      new SpoonRetract(loadingSpoon)
    ));
    */
  }
}
