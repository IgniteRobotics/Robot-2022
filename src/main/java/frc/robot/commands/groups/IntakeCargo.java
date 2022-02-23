// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.groups;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.indexer.IndexCargo;
import frc.robot.commands.intake.RunIntake;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCargo extends ParallelRaceGroup {
  private Intake m_intake;
  private Indexer m_indexer;

  /** Creates a new IntakeCargo. */
  public IntakeCargo(Intake intake, Indexer indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunIntake(intake, true),
      new IndexCargo(indexer)
    );
  }
}
