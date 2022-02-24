/***************************************************  
This command feeds cargo from the intake and ends when both slots are full.

***************************************************/

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexCargo extends CommandBase {
  private Indexer m_indexer;
  

  /** Creates a new IndexCargo. */
  public IndexCargo(Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_indexer = indexer;
    addRequirements(m_indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //run the belts unless both beam breaks are blocked.
    if (!m_indexer.getInitialIndexerBeamBreak() || !m_indexer.getKickupIndexerBeamBreak()){
      m_indexer.advanceBelt();
    }
    //run the kickup unless there's already a ball there.
    if(!m_indexer.getKickupIndexerBeamBreak()){
      m_indexer.advanceKickUp();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_indexer.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if both beam breaks are broken, indexer is full and we're finished.
    return m_indexer.getInitialIndexerBeamBreak() && m_indexer.getKickupIndexerBeamBreak();
  }
}
