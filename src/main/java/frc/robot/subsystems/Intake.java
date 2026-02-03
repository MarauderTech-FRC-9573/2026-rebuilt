package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase{
//Kraken aka talonFX
    private final TalonFX intakeMotor;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.intakeMotorID); 
    }

    public void run(double speed) {
        intakeMotor.set(speed);

    }

    @Override
    public void periodic() { 
        // This method will be called once per scheduler run 
        

    }
}
