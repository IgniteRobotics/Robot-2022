/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.igniterobotics.robotbase.preferences.DoublePreference;
import com.igniterobotics.robotbase.reporting.ReportingBoolean;
import com.igniterobotics.robotbase.reporting.ReportingLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class Intake extends SubsystemBase {
    public static final int CURRENT_LIMIT = 20;
    private final DoublePreference intakeSpeed = new DoublePreference("Intake Speed");

    private final CANSparkMax intakeMotor;
    private DoubleSolenoid intakePistonSolenoid;

    private boolean isExtended;

    private final ReportingBoolean extendedReporting = new ReportingBoolean("Intake Extended", ReportingLevel.TEST);

    /**
     * Creates a new Intake.
     */
    public Intake() {
        intakeMotor = new CANSparkMax(PortConstants.kIntakeMotorPort, MotorType.kBrushless);

        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);
        intakeMotor.setSmartCurrentLimit(CURRENT_LIMIT);

        isExtended = false;

        intakePistonSolenoid = new DoubleSolenoid(21, PneumaticsModuleType.REVPH,
                PortConstants.kIntakeSolenoidForwardPort, PortConstants.kIntakeSolenoidReversePort);

    }

    public void extendIntake() {
        isExtended = true;
        // retracting the piston actually extends the intake!
        extendedReporting.set(isExtended);
        intakePistonSolenoid.set(Value.kForward);
    }

    public void retractIntake() {
        isExtended = false;
        extendedReporting.set(isExtended);
        // extending the piston actually retracts the intake!
        intakePistonSolenoid.set(Value.kReverse);
    }
    // idk what that means haha

    public void toggleIntake() {
        if (isExtended) {
            retractIntake();
        } else {
            extendIntake();
        }
    }
    // intake go in and out or sumn?

    public void spin(double speed) {
        intakeMotor.set(speed);
    }

    public void spin() {
        this.spin(-Math.abs(intakeSpeed.getValue()));
    }

    public void outtake() {
        this.spin(Math.abs(intakeSpeed.getValue()));
    }

    public void stop() {
        intakeMotor.set(0);
        this.retractIntake();
    }

    public boolean isExtended() {
        extendedReporting.set(isExtended);
        return isExtended;
    }

    public boolean isRunning() {
        return intakeMotor.get() != 0.0;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
