// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ControllerRumble extends CommandBase {
  private XboxController controller;
  private int milliseconds;
  private long start;

  /** Creates a new ControllerRumble. */
  public ControllerRumble(XboxController controller, int milliseconds) {
    this.controller = controller;
    this.milliseconds = milliseconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.start = System.currentTimeMillis();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //this.controller.setRumble(RumbleType., 1);
    controller.setRumble(RumbleType.kLeftRumble, 1.0);
    controller.setRumble(RumbleType.kRightRumble, 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kLeftRumble, 0.0);
    controller.setRumble(RumbleType.kRightRumble, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() - start >= milliseconds;
  }
}
