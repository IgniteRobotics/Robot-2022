// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotStateController;
import frc.robot.subsystems.Indexer;

public class RunIndexerKickupDelay extends CommandBase {
  private Indexer indexer;

  private Supplier<Double> delaySupplier;

  private double delay;

  private double startTime;

  /** Creates a new RunIndexerKickupDelay. */
  public RunIndexerKickupDelay(Indexer indexer, Supplier<Double> delaySupplier) {
    this.indexer = indexer;
    this.delaySupplier = delaySupplier;

    // addRequirements(indexer);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    delay = delaySupplier.get();
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double passedTime = Timer.getFPGATimestamp() - startTime;

    indexer.advanceKickUp();

    if(passedTime >= delay) {
      indexer.advanceBelt();
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
