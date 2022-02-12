package frc.robot.commands.climber;

import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;

public class ClimbJoystick extends CommandBase {
  private Climber climb;
  private XboxController climbController;

  private final double effortLimit = 0.25;

  /** Creates a new ClimbUp. */
  public ClimbJoystick(XboxController climbController, Climber climb) {
    // Use addRequirements() here to declare subsystem dependencies.
     this.climb = climb;
     this.climbController = climbController;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double effort = climbController.getLeftX();
      double convertedEffort = (effort / 1) * effortLimit;
      climb.go(convertedEffort);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}