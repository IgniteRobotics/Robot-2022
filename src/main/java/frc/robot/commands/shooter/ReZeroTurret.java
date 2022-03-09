/************************************************* 
Move the to position X to move it to face directly to the rear.
Then reset encoders to 0 so we are now "centered"
***************************************************/


package frc.robot.commands.shooter;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;

public class ReZeroTurret extends CommandBase {

  private Turret m_turret;
  private Supplier<Double> offsetSupplier;

  /** Creates a new InitializeTurret. */
  public ReZeroTurret(Turret turret, Supplier<Double> offsetSupplier) {
    this.m_turret = turret;
    this.offsetSupplier = offsetSupplier;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_turret.gotoPostion(offsetSupplier.get().doubleValue());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (!interrupted){
      m_turret.resetEncoder();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_turret.isAtPosition();
  }
}
