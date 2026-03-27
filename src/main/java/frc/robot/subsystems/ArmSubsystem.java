package frc.robot.subsystems;


import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.ArmConstants.ARM_MAX_CURRENT;
import static frc.robot.Constants.ArmConstants.ARM_MOTOR_ID;

import java.time.format.ResolverStyle;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
    //Neo motor with SparkMarxController
    //intitalize
    private final SparkMax arm;
    public ArmSubsystem(){
        arm = new SparkMax(ARM_MOTOR_ID, MotorType.kBrushless);
        
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.smartCurrentLimit(ARM_MAX_CURRENT);

        arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void setArm(double speed){
        arm.set(speed);
    }

    public void stop(){
        arm.set(0.15);
    }
    
}