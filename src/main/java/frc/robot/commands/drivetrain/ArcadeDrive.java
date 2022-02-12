/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;

public class ArcadeDrive extends CommandBase { //TODO Figure out how to make a button trigger slow mode

    private Drivetrain m_driveTrain = null;
    private XboxController driverController = null;

    private boolean isSlowMode = false; //Figure out a button for this.
    private boolean isReversed = false;

    /**
     * Creates a new ArcadeDrive.
     */
    public ArcadeDrive(XboxController driveController, Drivetrain driveTrain) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.driverController = driveController;
        this.m_driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    //Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveTrain.arcadeDrive(getSpeed(), getRotation(), true);
        outputTelemetry();
    }

    private double getSpeed() {
        double speed = driverController.getLeftY() * (isReversed ? 1 : -1);
        // if(m_driveTrain.isSlowMode) {
        //   speed *= Constants.SLOW_MODE_SPEED_MODIFIER;
        // }
        return speed;
    }

    private double getRotation() {
        double rotation = driverController.getRightX();
        // if(m_driveTrain.isSlowMode) {
        //   rotation *= Constants.SLOW_MODE_SPEED_MODIFIER;
        // }
        return rotation;
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveTrain.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void setReversed(boolean b) {
        this.isReversed = b;
    }

    public void toggleReversed() {
        this.isReversed = !this.isReversed;
    }

    public void outputTelemetry() {
        SmartDashboard.putNumber("AD/Speed", this.getSpeed());
        SmartDashboard.putNumber("AD/Rotation", this.getRotation());
    }
}