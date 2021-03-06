// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class RunIndexerBelts extends CommandBase {
  private final Indexer indexer;
  private final boolean advance;

  /** Creates a new RunIndexerBelts. */
  public RunIndexerBelts(Indexer indexer, boolean advance) {
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
    } else {
      indexer.retreatBelt();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopBelt();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
