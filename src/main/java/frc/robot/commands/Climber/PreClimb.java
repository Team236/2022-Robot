// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Mast;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PreClimb extends SequentialCommandGroup {
  /** Creates a new ClimbSequence. */
  public PreClimb(Mast mast, Arm arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ArmPID(arm, ClimberConstants.ARM_TO_CLIMB, 1).withTimeout(3),
      new MastPID(mast, ClimberConstants.MAST_UP, 1).withTimeout(3)
    );
  }
}
