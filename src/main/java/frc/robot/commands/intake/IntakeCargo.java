// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCargo extends CommandBase {
  private Indexer indexer;
  private Intake intake;
  /** Creates a new IntakeCargo. */
  public IntakeCargo(Intake intake, Indexer indexer) {
    this.indexer = indexer;
    this.intake = intake;
    addRequirements(indexer, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!intake.isExtended()) {
      intake.extendIntake();
    }
    if (!indexer.getKickupIndexerBeamBreak()){
      indexer.advanceKickUp();
    }
    if (!indexer.getInitialIndexerBeamBreak() || !indexer.getKickupIndexerBeamBreak()){
      indexer.advanceBelt();
    }
    intake.spin();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopAll();
    intake.stop();
    indexer.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return indexer.getKickupIndexerBeamBreak() && indexer.getInitialIndexerBeamBreak();
  }
}
