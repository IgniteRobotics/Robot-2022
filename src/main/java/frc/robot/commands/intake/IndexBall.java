// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotStateController;
import frc.robot.subsystems.Indexer;

public class IndexBall extends CommandBase {
  private Indexer indexer;
  private RobotStateController robotState = RobotStateController.getInstance();
  


  /** Creates a new IntakeBall. */
  public IndexBall(Indexer indexer) {
    this.indexer = indexer;

    addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    indexer.indexBall();
    indexer.updateColorInfo();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexer.stopAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotState.isIndexerFull();
  }
}
