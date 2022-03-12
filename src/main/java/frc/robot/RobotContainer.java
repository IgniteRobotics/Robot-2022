// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.igniterobotics.robotbase.calc.InterCalculator;
import com.igniterobotics.robotbase.calc.InterParameter;
import com.igniterobotics.robotbase.preferences.DoublePreference;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.climber.ClimbDown;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.climber.RetractClimbMax;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.ArcadeSetDrive;
import frc.robot.commands.indexer.IndexBall;
import frc.robot.commands.indexer.RunIndexerAndKickup;
import frc.robot.commands.indexer.RunIndexerBelts;
import frc.robot.commands.intake.OuttakeIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.limelight.LimelightSetLed;
import frc.robot.commands.shooter.ReZeroTurret;
import frc.robot.commands.shooter.ResetTurretEncoder;
import frc.robot.commands.shooter.RunTurret;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.ShootSetVelocity;
import frc.robot.commands.shooter.TurretTarget;
import frc.robot.constants.PortConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private RobotStateController controller = RobotStateController.getInstance();

  private SendableChooser<Command> autonChooser = new SendableChooser<>();

  private DoublePreference shooterVelocityPreference = new DoublePreference("Shooter Set Velocity", 0);
  private DoublePreference shooterFenderLowPreference = new DoublePreference("FenderLow Velocity", 2500);
  private DoublePreference shooterFenderHighPreference = new DoublePreference("FenderHigh Velocity", 7000);
  private DoublePreference shooterEjectPreference = new DoublePreference("Eject Velocity", 9000);

  public static final double DEFAULT_HOOD = 180;

  private DoublePreference hoodPosition = new DoublePreference("Hood Set Position", 0);
  private DoublePreference initialTurretOffset = new DoublePreference("Initial Turret Offset", 0);
  private DoublePreference defaultTurrentPosition = new DoublePreference("Default Turret Position", 0);

  // controllers
  private XboxController m_driveController = new XboxController(PortConstants.DRIVER_CONTROLLER_PORT);
  private XboxController m_manipController = new XboxController(PortConstants.MANIPULATOR_CONTROLLER_PORT);

  // subsystems
  public final Drivetrain m_driveTrain = new Drivetrain();
  public final Intake m_intake = new Intake();
  public final Indexer m_indexer = new Indexer();
  public final Shooter m_shooter = new Shooter();
  public final Turret m_turret = new Turret();
  public final Hood m_hood = new Hood();
  public final Limelight m_limelight = new Limelight();
  public final Climber m_climber = new Climber();

  // comands
  private ResetTurretEncoder resetTurretEncoder = new ResetTurretEncoder(m_turret);
  private ArcadeDrive arcadeDriveCommand = new ArcadeDrive(m_driveController, m_driveTrain);
  private IndexBall indexBallCommand = new IndexBall(m_indexer);
  // private RetractIntake retractIntakeCommand = new RetractIntake(m_intake);
  private RunIntake runIntakeCommand = new RunIntake(m_intake, true);

  private ParallelRaceGroup outtakeSingleBall = new ParallelRaceGroup(new RunIndexerBelts(m_indexer, false),
      new OuttakeIntake(m_intake));
  private ShootSetVelocity shootVelocityCommand = new ShootSetVelocity(m_shooter, shooterVelocityPreference, false);
  // command group that runs the indexer and the intake until the indexer is full.
  private ParallelRaceGroup indexerIntakeGroup = new ParallelRaceGroup(new IndexBall(m_indexer),
      new RunIntake(m_intake, true));

  private SequentialCommandGroup autonCommandGroup = new SequentialCommandGroup(
    new ArcadeSetDrive(m_driveTrain, () -> 0.5).withTimeout(0.8),
    new ReZeroTurret(m_turret, defaultTurrentPosition).withTimeout(1),
    new TurretTarget(m_limelight, m_turret).withTimeout(1.5),
    createShootSetVelocity(
      () -> I_CALCULATOR.calculateParameter(m_limelight.getDistance()).vals[0],
      () -> 180.0).withTimeout(4)
  );

  private RetractClimbMax retractClimbMax = new RetractClimbMax(m_climber);
  private ClimbUp climbUp = new ClimbUp(m_climber);
  private ClimbDown climbDown = new ClimbDown(m_climber);

  private Command shootGroup = createShootSetVelocity(shooterVelocityPreference, () -> 180.0);

  private Command shootFenderLow = createShootSetVelocity(shooterFenderLowPreference, () -> 180.0);
  private Command shootFenderHigh = createShootSetVelocity(shooterFenderHighPreference, () -> 0.0);
  private Command shootEject = createShootSetVelocity(shooterEjectPreference, () -> 180.0);

  private Command shootInterpolated = createShootSetVelocity(
      () -> I_CALCULATOR.calculateParameter(m_limelight.getDistance()).vals[0],
      () -> 180.0);

  private JoystickButton btn_driveA = new JoystickButton(m_driveController, XboxController.Button.kA.value);
  private JoystickButton btn_driveB = new JoystickButton(m_driveController, XboxController.Button.kB.value);
  private JoystickButton btn_driveX = new JoystickButton(m_driveController, XboxController.Button.kX.value);
  private JoystickButton btn_driveY = new JoystickButton(m_driveController, XboxController.Button.kY.value);
  private JoystickButton bumper_driveR = new JoystickButton(m_driveController,
      XboxController.Button.kRightBumper.value);
  private JoystickButton bumper_driveL = new JoystickButton(m_driveController, XboxController.Button.kLeftBumper.value);

  private JoystickButton btn_manipA = new JoystickButton(m_manipController, XboxController.Button.kA.value);
  private JoystickButton btn_manipX = new JoystickButton(m_manipController, XboxController.Button.kX.value);
  private JoystickButton btn_manipY = new JoystickButton(m_manipController, XboxController.Button.kY.value);
  private JoystickButton btn_manipB = new JoystickButton(m_manipController, XboxController.Button.kB.value);

  private JoystickButton bumper_manipR = new JoystickButton(m_manipController,
      XboxController.Button.kRightBumper.value);
  private JoystickButton bumper_manipL = new JoystickButton(m_manipController, XboxController.Button.kLeftBumper.value);

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    configureSubsystemCommands();

    autonChooser.addOption("NO AUTON", null);
    autonChooser.addOption("Shoot and Drive", autonCommandGroup);

    SmartDashboard.putData(resetTurretEncoder);
    SmartDashboard.putData(retractClimbMax);
    SmartDashboard.putData(climbUp);
    SmartDashboard.putData(climbDown);
    SmartDashboard.putData(autonChooser);
  }

  private void configureButtonBindings() {
    bumper_driveR.whileHeld(indexerIntakeGroup, true).whenReleased(new IndexBall(m_indexer).withTimeout(1));
    bumper_driveL.whileHeld(new RunIntake(m_intake, false));

    btn_driveA.whenHeld(shootFenderLow);
    btn_driveY.whileHeld(shootFenderHigh);
    btn_driveB.whenHeld(shootInterpolated);
    btn_driveX.whenHeld(shootEject);

    btn_manipA.whileHeld(new TurretTarget(m_limelight, m_turret));
    btn_manipY.whileHeld(climbUp);
    btn_manipB.whileHeld(climbDown);
    btn_manipX.whileHeld(shootGroup);

    bumper_manipR.whileHeld(new OuttakeIntake(m_intake));
    bumper_manipL.whileHeld(outtakeSingleBall);
  }

  /**
   * Use this method to configure default commands for subsystems
   */
  private void configureSubsystemCommands() {
    m_driveTrain.setDefaultCommand(arcadeDriveCommand);
    m_hood.setDefaultCommand(new SetHoodPosition(m_hood, () -> DEFAULT_HOOD));
    // TODO change to this default command after we verify soft limits are working
    m_turret.setDefaultCommand(new ReZeroTurret(m_turret, defaultTurrentPosition));
    m_limelight.setDefaultCommand(new LimelightSetLed(m_limelight, () -> true));
    // m_intake.setDefaultCommand(retractIntakeCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // TODO: Replace with real auton command. This is just here so it doesn't whine.
    return autonChooser.getSelected();
  }

  // DISTANCE, VELOCITY, HOOD_ANGLE
  public static final InterCalculator I_CALCULATOR = new InterCalculator(
      new InterParameter(1.35, 6400, 180),
      new InterParameter(1.94, 7200, 180),
      new InterParameter(2.6, 7500, 180),
      new InterParameter(2.8, 7900, 180),
      new InterParameter(3.51, 9300, 180));

  private Command createShootSetVelocity(Supplier<Double> velocity, Supplier<Double> hoodAngle) {
    return new ParallelDeadlineGroup(
        new SequentialCommandGroup(
            new SetHoodPosition(m_hood, hoodAngle).withInterrupt(() -> hoodAngle.get() == DEFAULT_HOOD).withTimeout(1),
            new WaitUntilCommand(m_shooter::isSetpointMet).andThen(new WaitCommand(0.25)),
            new RunIndexerAndKickup(m_indexer, true, 3)),
        new ShootSetVelocity(m_shooter, velocity, false));
  }
}
