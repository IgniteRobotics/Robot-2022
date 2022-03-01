// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import javax.swing.text.StyledEditorKit.BoldAction;

import com.igniterobotics.robotbase.reporting.ReportingBoolean;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingString;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotStateController;
import frc.robot.subsystems.Intake;

//  RunIntake
//
//  Designed to be triggered with OnPressed.  First press of button
//  extends and runs the intake.  Second press of button retracts and
//  stops the intake
//
//  If the intake is retracted with the indexer full, any attempt
//  to run the command will do nothing and just rumble the controller
//  to let the driver know indexer is full and can't intake any more cargo
//

public class RunIntake extends CommandBase {
  private Intake intake;
  // true == intake. false == outtake
  private boolean direction = true;
  private RobotStateController robotState;
  private boolean isFinished = false;

  /** Creates a new IntakeCargo. */
  public RunIntake(Intake intake, boolean direction) {
    this.intake = intake;
    this.direction = direction;
    addRequirements(intake);
    robotState = RobotStateController.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (robotState.IsIndexerFull()) {
      return;
    } else {
      if (!intake.isExtended()) {
        intake.extendIntake();
      }

      intake.spin();
    }
    // if (this.direction){
    // intake.spin(Intake.INTAKE_SPEED);
    // } else {
    // intake.spin(Intake.OUTTAKE_SPEED);
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.retractIntake();
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
