// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilStable extends CommandBase {
  private DoubleSupplier valueSupplier;

  private final double maxDiff;
  private final int minFrames;

  private double lastValue;
  private int stableFrames;

  /** Creates a new WaitUntilStable. */
  public WaitUntilStable(DoubleSupplier valueSupplier, double maxDiff, int minFrames) {
    this.valueSupplier = valueSupplier;
    this.maxDiff = maxDiff;
    this.minFrames = minFrames;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastValue = valueSupplier.getAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentValue = valueSupplier.getAsDouble();
    double diff = Math.abs(currentValue - lastValue);

    if(diff <= maxDiff) {
      stableFrames++;
    } else {
      stableFrames = 0;
    }

    lastValue = currentValue;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stableFrames >= minFrames;
  }
}
