// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class RunIndexerAndKickup extends CommandBase {
  private final Indexer indexer;
  private final boolean advance;

  /** Creates a new RunIndexerBelts. */
  public RunIndexerAndKickup(Indexer indexer, boolean advance) {
    this.indexer = indexer;
    this.advance = advance;

    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (advance) {
      indexer.advanceBelt();
      indexer.advanceKickUp();
    } else {
      indexer.retreatBelt();
      indexer.retreatKickup();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopBelt();
    indexer.stopKickup();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
