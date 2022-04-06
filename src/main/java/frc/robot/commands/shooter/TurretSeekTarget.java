// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class TurretSeekTarget extends CommandBase {
  private Limelight limelight;
  private Turret turret;

  private boolean positiveDirection;
  private double seekEffort = 0.7;

  /** Creates a new TurretSeekTarget. */
  public TurretSeekTarget(Limelight limelight, Turret turret) {
    this.limelight = limelight;
    this.turret = turret;

    addRequirements(turret, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(positiveDirection) {
      turret.runTurret(seekEffort);
    } else {
      turret.runTurret(-seekEffort);
    }

    if(turret.isForwardSoftReached()) {
      positiveDirection = false;
    } else if(turret.isReverseSoftReached()) {
      positiveDirection = true;
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
    return limelight.getTv();
  }
}
