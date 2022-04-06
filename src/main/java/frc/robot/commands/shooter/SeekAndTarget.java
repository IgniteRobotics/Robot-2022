// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Turret;

public class SeekAndTarget extends CommandBase {
  private Limelight limelight;
  private Turret turret;

  private boolean positiveDirection;
  private double seekEffort = 0.7;

  private static final double kP_TURN = 0.05;
  private static final double kI_TURN = 0;
  private static final double kD_TURN = 0;

  private PIDController targetPidController;
  
  /** Creates a new SeekAndTarget. */
  public SeekAndTarget(Limelight limelight, Turret turret) {
    this.limelight = limelight;
    this.turret = turret;

    targetPidController = new PIDController(kP_TURN, kI_TURN, kD_TURN);
    targetPidController.setSetpoint(0);

    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.getTv()) {
      target();
    } else {
      seek();
    }
  }

  private void target() {
    double output = targetPidController.calculate(limelight.getTx());
    turret.runTurret(output);
  }

  private void seek() {
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
    return false;
  }
}
