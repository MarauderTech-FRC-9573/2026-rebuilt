package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_ID;
import static frc.robot.Constants.FuelConstants.LAUNCHER_MOTOR_CURRENT_LIMIT;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    
    private TalonFX intake;


    public ShooterSubsystem() {
        intake = new TalonFX(LAUNCHER_MOTOR_ID);
         
        //Added current Limit
        var currentLimits = new CurrentLimitsConfigs(); 
        currentLimits.SupplyCurrentLimit = LAUNCHER_MOTOR_CURRENT_LIMIT;
        intake.getConfigurator().apply(currentLimits);
       }

    public void run(double speed) {
            intake.set(speed);
}


    public void stop() {
        intake.set(0);
    }

}