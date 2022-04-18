// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//Makes this file recognized as commands used by the climber.
package frc.robot.commands.climber;

//Imports WPIlibs Xbox controller commands, their command base and the Climber subsystem.
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

//Creates a new command, ClimbDownEngage, which uses other commands located in the Climber subsystem, and is used by RobotContainer.java. TODO Ask if we need this whole file, as RobotContainer.java imports the climber subsystem anyway.
public class ClimbDownEngage extends CommandBase {
  //Creates Variables for the climber and joystick. TODO ask if this is true.
  private Climber climber;
  private XboxController joystick;

  /** Creates a new ClimbDownEngage. */
  public ClimbDownEngage(Climber climber, XboxController joystick) {
    //Recognizes the climber used on line 19 is the same as the climber used on line 15. Does the same for the joystick. (Lines 19 and 16)
    this.climber = climber;
    this.joystick = joystick;
    //Makes this command a dependancy of the climber subsystem
    addRequirements(climber);
  }

  // Called when the command is initially scheduled, It doesn't seem we use this. TODO Ask about the purpose for this command, and if we need it.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled, tells the climber to go down until engages with the robot body.
  @Override
  public void execute() {
    climber.goDownEngage();
  }

  // Called once the command ends or is interrupted, tells the climber to stop.
  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}