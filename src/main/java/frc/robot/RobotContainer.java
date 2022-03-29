// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Paths;
import java.util.List;
import java.util.function.Supplier;

import com.igniterobotics.robotbase.calc.InterCalculator;
import com.igniterobotics.robotbase.calc.InterParameter;
import com.igniterobotics.robotbase.preferences.DoublePreference;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.igniterobotics.robotbase.reporting.ReportingNumber;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.climber.ClimbDown;
import frc.robot.commands.climber.ClimbUp;
import frc.robot.commands.climber.ForwardSecondaryClimber;
import frc.robot.commands.climber.RetractClimbMax;
import frc.robot.commands.climber.ReverseSecondaryClimber;
import frc.robot.commands.drivetrain.ArcadeDrive;
import frc.robot.commands.drivetrain.ArcadeSetDrive;
import frc.robot.commands.drivetrain.RamseteTrajectoryCommand;
import frc.robot.commands.indexer.IndexBall;
import frc.robot.commands.indexer.RunIndexerBelts;
import frc.robot.commands.indexer.RunIndexerKickupDelay;
import frc.robot.commands.intake.OuttakeIntake;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.limelight.LimelightSetLed;
import frc.robot.commands.shooter.ReZeroTurret;
import frc.robot.commands.shooter.ResetTurretEncoder;
import frc.robot.commands.shooter.SetHoodPosition;
import frc.robot.commands.shooter.ShootSetVelocity;
import frc.robot.commands.shooter.TurretSeekTarget;
import frc.robot.commands.shooter.TurretTarget;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PortConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SecondaryClimber;
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

    private DoublePreference velocityOffset = new DoublePreference("VELOCITY OFFSET", 0);

    private DoublePreference beltDelayPreference = new DoublePreference("Belt Delay", 1);

    public static final double DEFAULT_HOOD = 180;

    private DoublePreference hoodPosition = new DoublePreference("Hood Set Position", 0);
    private DoublePreference initialTurretOffset = new DoublePreference("Initial Turret Offset", 0);
    private DoublePreference defaultTurrentPosition = new DoublePreference("Default Turret Position", 0);

    public final ReportingNumber interpolatedRPMReporter = new ReportingNumber("Interpolated Velocity",
            ReportingLevel.COMPETITON);

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
    private final SecondaryClimber m_secondaryClimber = new SecondaryClimber();

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

    private SequentialCommandGroup driveAndShoot = new SequentialCommandGroup(
            new ArcadeSetDrive(m_driveTrain, () -> 0.5).withTimeout(0.8),
            new ReZeroTurret(m_turret, defaultTurrentPosition).withTimeout(1),
            new TurretTarget(m_limelight, m_turret).withTimeout(1.5),
            createShootSetVelocity(
                    this::getCalculatedVelocity,
                    () -> 180.0,
                    () -> 0.0).withTimeout(4));

    private SequentialCommandGroup twoBallAuton = new SequentialCommandGroup(
            new ParallelCommandGroup(
                    new ArcadeSetDrive(m_driveTrain, () -> 0.2).withTimeout(1.7),
                    new ParallelRaceGroup(new IndexBall(m_indexer), new RunIntake(m_intake, true)),
                    new ReZeroTurret(m_turret, defaultTurrentPosition)).withTimeout(2),
            new TurretTarget(m_limelight, m_turret).withTimeout(2.2),
            createShootSetVelocity(
                    this::getCalculatedVelocity,
                    () -> 180.0,
                    () -> 0.0).withTimeout(5));

    private RetractClimbMax retractClimbMax = new RetractClimbMax(m_climber);
    private ClimbUp climbUp = new ClimbUp(m_climber);
    private ClimbDown climbDown = new ClimbDown(m_climber);

    private CommandGroupBase shootTest = createShootSetVelocity(shooterVelocityPreference, hoodPosition,
            beltDelayPreference);

    private Command shootFenderLow = createShootSetVelocity(shooterFenderLowPreference, () -> 180.0,
            beltDelayPreference);
    private Command shootFenderHigh = createShootSetVelocity(shooterFenderHighPreference, () -> 0.0,
            beltDelayPreference);
    private Command shootEject = createShootSetVelocity(shooterEjectPreference, () -> 180.0, beltDelayPreference);

    // private CommandBase turretTarget = new TurretTarget(m_limelight, m_turret);
    private CommandBase turretSeekAndTarget = new SequentialCommandGroup(
            new TurretSeekTarget(m_limelight, m_turret),
            new TurretTarget(m_limelight, m_turret));

    private CommandBase setHoodPosition = new SetHoodPosition(m_hood, hoodPosition);

    private CommandBase shootInterpolated = createShootSetVelocity(
            this::getCalculatedVelocity,
            this::getCalculatedHood,
            beltDelayPreference);

    private JoystickButton btn_driveA = new JoystickButton(m_driveController, XboxController.Button.kA.value);
    private JoystickButton btn_driveB = new JoystickButton(m_driveController, XboxController.Button.kB.value);
    private JoystickButton btn_driveX = new JoystickButton(m_driveController, XboxController.Button.kX.value);
    private JoystickButton btn_driveY = new JoystickButton(m_driveController, XboxController.Button.kY.value);
    private JoystickButton bumper_driveR = new JoystickButton(m_driveController,
            XboxController.Button.kRightBumper.value);
    private JoystickButton bumper_driveL = new JoystickButton(m_driveController,
            XboxController.Button.kLeftBumper.value);

    private JoystickButton btn_manipA = new JoystickButton(m_manipController, XboxController.Button.kA.value);
    private JoystickButton btn_manipX = new JoystickButton(m_manipController, XboxController.Button.kX.value);
    private JoystickButton btn_manipY = new JoystickButton(m_manipController, XboxController.Button.kY.value);
    private JoystickButton btn_manipB = new JoystickButton(m_manipController, XboxController.Button.kB.value);

    private POVButton dpad_driverUp = new POVButton(m_driveController, 0);

    private POVButton dpad_manipUp = new POVButton(m_manipController, 0);
    private POVButton dpad_manipDown = new POVButton(m_manipController, 180);

    private JoystickButton bumper_manipR = new JoystickButton(m_manipController,
            XboxController.Button.kRightBumper.value);
    private JoystickButton bumper_manipL = new JoystickButton(m_manipController,
            XboxController.Button.kLeftBumper.value);

    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        configureSubsystemCommands();

        indexerIntakeGroup.addRequirements(m_turret);

        autonChooser.addOption("NO AUTON", null);
        autonChooser.addOption("Drive and Shoot", driveAndShoot);
        autonChooser.addOption("Two Ball", twoBallAuton);

        SmartDashboard.putData(resetTurretEncoder);
        SmartDashboard.putData(retractClimbMax);
        SmartDashboard.putData(climbUp);
        SmartDashboard.putData(climbDown);
        SmartDashboard.putData("Shoot Set Velocity", shootTest);
        SmartDashboard.putData(turretSeekAndTarget);
        SmartDashboard.putData(setHoodPosition);
        SmartDashboard.putData("Shoot Interpolated", shootInterpolated);

        SmartDashboard.putData(autonChooser);
    }

    private void configureButtonBindings() {
        bumper_driveR.whileHeld(indexerIntakeGroup, true).whenReleased(new IndexBall(m_indexer).withTimeout(1));
        bumper_driveL.whileHeld(new RunIntake(m_intake, false));

        btn_driveA.whenHeld(shootFenderLow);
        btn_driveY.whileHeld(shootFenderHigh);
        btn_driveB.whenHeld(shootInterpolated);
        btn_driveX.whenHeld(shootEject);

        btn_manipA.whileHeld(turretSeekAndTarget);
        btn_manipY.whenHeld(climbUp);
        btn_manipB.whenHeld(climbDown);
        btn_manipX.whileHeld(shootTest);

        bumper_manipR.whileHeld(new OuttakeIntake(m_intake));
        bumper_manipL.whileHeld(outtakeSingleBall);

        dpad_manipUp.whenPressed(new ForwardSecondaryClimber(m_secondaryClimber));
        dpad_manipDown.whenPressed(new ReverseSecondaryClimber(m_secondaryClimber));

        dpad_driverUp.whenPressed(() -> arcadeDriveCommand.setTurboMode(true))
                .whenReleased(() -> arcadeDriveCommand.setTurboMode(false));
    }

    /**
     * Use this method to configure default commands for subsystems
     */
    private void configureSubsystemCommands() {
        m_driveTrain.setDefaultCommand(arcadeDriveCommand);
        m_hood.setDefaultCommand(new SetHoodPosition(m_hood, this::getCalculatedHood));
        // TODO change to this default command after we verify soft limits are working
        // m_turret.setDefaultCommand(new ContinuousConditionalCommand(
        // new ReZeroTurret(m_turret, defaultTurrentPosition),
        // new SequentialCommandGroup(
        // new TurretSeekTarget(m_limelight, m_turret),
        // new TurretTarget(m_limelight, m_turret).perpetually()
        // ),
        // controller::isIndexerEmpty
        // ).perpetually());
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
        // return autonChooser.getSelected();
        Trajectory straightTrajectory = loadTrajectory("Forward3m");

        Command command = new RamseteTrajectoryCommand(m_driveTrain, straightTrajectory)
                .andThen(() -> m_driveTrain.tankDriveVolts(0, 0));

        return command;

        // Command driveBackAndIntake = new ParallelCommandGroup(command,
        //         new ParallelRaceGroup(new IndexBall(m_indexer), new RunIntake(m_intake, true)));

        // return new SequentialCommandGroup(
        //         driveBackAndIntake,
        //         new TurretTarget(m_limelight, m_turret).withTimeout(2.2),
        //         createShootSetVelocity(this::getCalculatedVelocity, () -> 180.0, () -> 0.0).withTimeout(5));
    }

    // DISTANCE, VELOCITY, HOOD_ANGLE
    public static final InterCalculator I_CALCULATOR = new InterCalculator(
            new InterParameter(1.32, 7700, 0),
            new InterParameter(1.7, 7700, 30),
            new InterParameter(1.7, 7700, 30),
            new InterParameter(2, 7700, 60),
            new InterParameter(2.2, 7150, 180),
            new InterParameter(2.5, 7250, 180),
            new InterParameter(2.7, 7350, 180),
            new InterParameter(2.95, 7550, 180),
            new InterParameter(3.2, 7750, 180),
            new InterParameter(3.45, 8100, 180),
            new InterParameter(3.6, 8550, 180),
            new InterParameter(3.8, 8650, 180),
            new InterParameter(4.0, 8700, 180),
            new InterParameter(4.2, 8725, 180),
            new InterParameter(4.3, 9500, 180),
            new InterParameter(4.35, 9775, 180),
            new InterParameter(4.4, 10000, 180));

    public double getCalculatedVelocity() {
        double calculated = I_CALCULATOR.calculateParameter(m_limelight.getDistanceAverage()).vals[0]
                + velocityOffset.getValue();
        interpolatedRPMReporter.set(calculated);

        return calculated;
    }

    public double getCalculatedHood() {
        if (!m_limelight.getTv()) {
            return 180;
        } else {
            return I_CALCULATOR.calculateParameter(m_limelight.getDistance()).vals[1];
        }
    }

    private CommandGroupBase createShootSetVelocity(Supplier<Double> velocity, Supplier<Double> hoodAngle,
            Supplier<Double> beltDelay) {
        return new ParallelCommandGroup(
                new SequentialCommandGroup(
                        new SetHoodPosition(m_hood, hoodAngle).withInterrupt(() -> hoodAngle.get() == DEFAULT_HOOD)
                                .withTimeout(1),
                        new WaitUntilCommand(m_shooter::isSetpointMet).andThen(new WaitCommand(0.5)),
                        new RunIndexerKickupDelay(m_indexer, beltDelay)),
                new ShootSetVelocity(m_shooter, velocity, false));
    }

    public static Trajectory loadTrajectory(String trajectoryName) {
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath()
                    .resolve(Paths.get("paths", "output", trajectoryName + ".wpilib.json")));
            return trajectory;
        } catch (IOException e) {
            DriverStation.reportError("Failed to load auto trajectory: " + trajectoryName, false);
            return null;
        }
    }
}
