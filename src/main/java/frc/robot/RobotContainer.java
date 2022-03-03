// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.indexer.IndexBall;
import frc.robot.commands.indexer.RunIndexerAndKickup;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.ShootBall;
import frc.robot.constants.PortConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  //controllers
  private XboxController m_driveController = new XboxController(PortConstants.DRIVER_CONTROLLER_PORT);

  //subsystems
  private Drivetrain m_driveTrain = new Drivetrain();
  private Intake m_intake = new Intake();
  private Indexer m_indexer = new Indexer();
  private Shooter m_shooter = new Shooter();

  //comands
  private ArcadeDrive arcadeDriveCommand = new ArcadeDrive(m_driveController, m_driveTrain);
  // private RetractIntake retractIntakeCommand = new RetractIntake(m_intake);
  RobotStateController controller = RobotStateController.getInstance();
  private RunIntake runIntakeCommand = new RunIntake(m_intake, true);
  private IndexBall indexBallCommand = new IndexBall(m_indexer);

  private ShootBall shootBallCommand = new ShootBall(m_shooter);

  private ParallelRaceGroup indexerIntakeGroup = new ParallelRaceGroup(new IndexBall(m_indexer), new RunIntake(m_intake, true));
  private ParallelRaceGroup shootGroup = new ParallelRaceGroup(new SequentialCommandGroup(new WaitCommand(0.25), new RunIndexerAndKickup(m_indexer, true)), new ShootBall(m_shooter).withTimeout(1.5));

  private JoystickButton btn_driveA = new JoystickButton(m_driveController, XboxController.Button.kA.value);
  private JoystickButton btn_driveB = new JoystickButton(m_driveController, XboxController.Button.kB.value);
  private JoystickButton btn_driveX = new JoystickButton(m_driveController, XboxController.Button.kX.value);
  private JoystickButton bumper_driveR = new JoystickButton(m_driveController, XboxController.Button.kRightBumper.value);   
  private JoystickButton bumper_driveL = new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value);   

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    bumper_driveR.whileHeld(indexerIntakeGroup, true);
    bumper_driveL.whileHeld(new RunIntake(m_intake, false));
    btn_driveA.whileHeld(shootBallCommand);
    btn_driveB.whenHeld(shootGroup);
  }

  /**
   * Use this method to configure default commands for subsystems
   */
  private void configureSubsystemCommands() {
    m_driveTrain.setDefaultCommand(arcadeDriveCommand);
    // m_intake.setDefaultCommand(retractIntakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //TODO:  Replace with real auton command.  This is just here so it doesn't whine.
    return new ExampleCommand(new ExampleSubsystem());
  }
}
