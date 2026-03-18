package frc.robot.subsystems;

import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_CURRENT_LIMIT;
import static frc.robot.Constants.FuelConstants.FEEDER_MOTOR_ID;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
    private final SparkMax feederMotor;

    public FeederSubsystem(){
        
        feederMotor = new SparkMax(FEEDER_MOTOR_ID, MotorType.kBrushless);

        //config for the feeder motor
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT);

        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /*
     * Runs the motor at specified speed between -1 and 1
     * @param speed the speed the motor will be set to run at
    */
    public void setFeeder(double speed ){
        feederMotor.set(speed);
    }

    //Stops the feeder motor
    public void stop(){
        feederMotor.set(0);
    }
    
}
