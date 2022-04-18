// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Makes this file recognized as commands used by the climber
package frc.robot.commands.climber;

//Imports WPIlibs command base and the Climber subsystem
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

//Creates a new command, ClimbDown, which uses other commands located in the Climber subsystem, and is used by RobotContainer.java. TODO Ask if we need this whole file, as RobotContainer.java imports the climber subsystem anyway.
public class ClimbDown extends CommandBase {
  //Creates a Variable for the climb. TODO ask if this is true.
  private Climber climb;
  /** Creates a new ClimbDown. */
  public ClimbDown(Climber climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    //Recognizes the climb used on line 16 is the same as the climb used on line 14
    this.climb = climb;
  }

  //Called when the command is initially scheduled, It doesn't seem we use this. TODO Ask about the purpose for this command, and if we need it.
  @Override
  public void initialize() {}

  //Called every time the scheduler runs while the command is scheduled, tells the climb to go down.
  @Override
  public void execute() {
    climb.goDown();
  }

  //Called once the command ends or is interrupted, tells the climb to stop.
  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  //Used to end the command, Checks if the climb subsystems limit has been met, and if it has been met, returns true. TODO See if this is right.
  @Override
  public boolean isFinished() {
    return climb.isClimbLimitMet();
  }
}