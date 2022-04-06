
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SecondaryClimber;

public class ForwardSecondaryClimber extends CommandBase {
  private Climber climber;
  private SecondaryClimber secondClimber;

  /** Creates a new DeploySecondaryClimber. */
  public ForwardSecondaryClimber(SecondaryClimber secondClimber, Climber climber) {
    this.climber = climber;
    this.secondClimber = secondClimber;

    addRequirements(secondClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(climber.wasDeployed()) {
      secondClimber.forward();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
