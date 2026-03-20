package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_ID;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    private TalonFX intake;


    public ShooterSubsystem() {
        intake = new TalonFX(LAUNCHER_MOTOR_ID);
    }

    public void run(double speed) {
            intake.set(speed);
}


    public void stop() {
        intake.set(0);
    }

}