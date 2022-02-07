/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class Intake extends SubsystemBase {

    private static final double INTAKE_SPEED = 0.5;

    private final CANSparkMax intakeMotor;
    private DoubleSolenoid intakePistonSolenoid;

    private boolean isExtended;

    /**
     * Creates a new Intake.
     */
    public Intake() {
        intakeMotor = new CANSparkMax(PortConstants.kIntakeMotorPort, MotorType.kBrushless); 

        intakeMotor.setInverted(false);
        intakeMotor.setIdleMode(IdleMode.kBrake);

        isExtended = false;

        intakePistonSolenoid = new DoubleSolenoid(44, PneumaticsModuleType.REVPH, PortConstants.kIntakeSolenoidForwardPort, PortConstants.kIntakeSolenoidReversePort);
   
      }

    public void extendIntake() {
        isExtended = true;
        intakePistonSolenoid.set(Value.kForward);
    }

    public void retractIntake() {
        isExtended = false;
        intakePistonSolenoid.set(Value.kReverse);
    }
    //idk what that means haha 

    public void toggleIntake() {
        if (isExtended) {
            retractIntake();
        } else {
            extendIntake();
        }
    }
//intake go in and out or sumn?

    public void spin(double speed) {
        intakeMotor.set(speed);
    }

    public void spin(){
        this.spin(INTAKE_SPEED);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    public boolean isExtended() {
        return isExtended;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
