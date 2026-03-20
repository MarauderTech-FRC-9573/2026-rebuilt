package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.Constants.FuelConstants.*;
import static frc.robot.Constants.IntakeConstants.INTAKE_MOTOR_ID;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{
    
    private final SparkMax intakeMotor;

    public IntakeSubsystem(){
        
        intakeMotor = new SparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);

        //config for the shooter motor
        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        intakeConfig.inverted(true);

        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /*
     * Runs the motor at specified speed between -1 and 1
     * @param   speed   the speed the motor will be set to run at
    */
    public void setIntake(double speed ){
        intakeMotor.set(speed);
    }

    //Stops the shooter motor
    public void stop(){
        intakeMotor.set(0);
    }

}
