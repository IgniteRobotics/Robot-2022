// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.igniterobotics.robotbase.reporting.ReportingBoolean;
import com.igniterobotics.robotbase.reporting.ReportingLevel;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilStable extends CommandBase implements BooleanSupplier {
  private DoubleSupplier valueSupplier;

  private final double maxDiff;
  private final int minFrames;

  private double lastValue;
  private int stableFrames;

  private boolean isSensor = false;

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

    System.out.println(stableFrames);

    lastValue = currentValue;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(isSensor) return false;
    return stableFrames >= minFrames;
  }

  @Override
  public boolean getAsBoolean() {
    return stableFrames >= minFrames;
  }

  public WaitUntilStable asSensor() {
    this.isSensor = true;
    return this;
  }
}
