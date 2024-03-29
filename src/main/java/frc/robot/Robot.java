// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private PneumaticHub pneumaticHub = new PneumaticHub();
  private Compressor compressor = new Compressor(21, PneumaticsModuleType.REVPH);

  private Thread visionThread;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    pneumaticHub.enableCompressorDigital();
    pneumaticHub.enableCompressorAnalog(100, 120);
    compressor.enableDigital();
    m_robotContainer.m_driveTrain.setNeutralMode(NeutralMode.Coast);
    if (Robot.isReal()){
      final UsbCamera camera = CameraServer.startAutomaticCapture();

      visionThread = new Thread(() -> {
        CvSink cvSink = CameraServer.getVideo();
        CvSource outputStream = CameraServer.putVideo("Driver Camera", 600, 800);

        Mat imageMat = new Mat();

        Scalar color = new Scalar(255, 0, 0);

        while(!Thread.interrupted()) {
          if(cvSink.grabFrame(imageMat) == 0) continue;

          Imgproc.circle(imageMat, new Point(imageMat.width() / 2, imageMat.height() / 2), 5, color);

          outputStream.putFrame(imageMat);
        }
      });

      visionThread.start();
    }
    m_robotContainer.stableSensor.schedule();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double calculatedVelocity = m_robotContainer.getCalculatedVelocity();
    m_robotContainer.interpolatedRPMReporter.set(calculatedVelocity);
    m_robotContainer.interpolatedHoodReporter.set(m_robotContainer.getCalculatedHood());
    m_robotContainer.isVelocityMet.set(m_robotContainer.m_shooter.isSetpointMet());

    m_robotContainer.isLimelightStable.set(m_robotContainer.stableSensor.getAsBoolean());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    RobotStateController robotState = RobotStateController.getInstance();
    robotState.reset();
    m_robotContainer.m_driveTrain.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.m_driveTrain.setNeutralMode(NeutralMode.Coast);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_driveTrain.setNeutralMode(NeutralMode.Brake);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    m_robotContainer.m_driveTrain.setNeutralMode(NeutralMode.Brake);
    m_robotContainer.m_driveTrain.setFollowerStatus();
    
    m_robotContainer.m_climber.reset();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
