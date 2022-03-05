// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.sound.sampled.LineEvent;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretTarget extends PIDCommand {
  private Limelight limelight;

  private static final double kP_TURN = 0;
  private static final double kI_TURN = 0;
  private static final double kD_TURN = 0;

  /** Creates a new TargetTurret. */
  public TurretTarget(Limelight limelight) {
    super(
        new PIDController(kP_TURN, kI_TURN, kD_TURN),
        () -> limelight.getTx(),
        0,
        output -> {
          // Use the output here
        });

    addRequirements(limelight);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}
