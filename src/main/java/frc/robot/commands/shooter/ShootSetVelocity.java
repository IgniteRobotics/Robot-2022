// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotStateController;
import frc.robot.subsystems.Shooter;

public class ShootSetVelocity extends CommandBase {
  private RobotStateController stateController = RobotStateController.getInstance();

  private Shooter shooter;
  private Supplier<Double> vSupplier;
  private boolean autoEnd = true;

  /** Creates a new ShootSetVelocity. */
  public ShootSetVelocity(Shooter shooter, Supplier<Double> vSupplier) {
    this.shooter = shooter;
    this.vSupplier = vSupplier;

    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ShootSetVelocity(Shooter shooter, Supplier<Double> vSupplier, boolean autoEnd) {
    this(shooter, vSupplier);
    this.autoEnd = autoEnd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.runFeed();
    shooter.runVelocity(vSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    stateController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stateController.isBreaksClear() && autoEnd;
  }
}
