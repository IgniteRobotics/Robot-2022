// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class RunTurret extends CommandBase {
  private Turret turret;
  private Supplier<Double> speedSupplier;

  /** Creates a new RunTurret. */
  public RunTurret(Turret turret, Supplier<Double> speedSupplier) {
    this.turret = turret;
    this.speedSupplier = speedSupplier;
    
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(speedSupplier.get());
    if(Math.abs(speedSupplier.get()) <= 0.1) {
      turret.stop();
    } else {
      turret.runTurret(speedSupplier.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
