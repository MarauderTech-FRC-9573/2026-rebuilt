package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Constants.FuelConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    private final SparkMax shooterMotor;

    public ShooterSubsystem(){
        
        shooterMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

        //config for the shooter motor
        SparkMaxConfig shootConfig = new SparkMaxConfig();
        shootConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);

        shooterMotor.configure(shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /*
     * Runs the motor at specified speed between -1 and 1
     * @param   speed   the speed the motor will be set to run at
    */
    public void setShooter(double speed ){
        shooterMotor.set(speed);
    }

    //Stops the shooter motor
    public void stop(){
        shooterMotor.set(0);
    }

}
