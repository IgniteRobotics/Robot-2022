// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class PassiveVelocity extends CommandBase {
  private Shooter shooter;
  private BooleanSupplier condition;
  private Supplier<Double> velocity;

  /** Creates a new PassiveVelocity. */
  public PassiveVelocity(Shooter shooter, BooleanSupplier condition, Supplier<Double> velocity) {
    this.shooter = shooter;
    this.condition = condition;
    this.velocity = velocity;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(condition.getAsBoolean()) {
      shooter.runVelocity(velocity.get());
    } else {
      shooter.stop();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
