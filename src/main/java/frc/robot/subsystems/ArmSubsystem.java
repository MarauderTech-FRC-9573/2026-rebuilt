package frc.robot.subsystems;


import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    private final PIDController armPidController;
    private double manualSpeed;
    private boolean manualOverride;
    private double targetPosition;

public ArmSubsystem(){
    arm = new SparkMax(Constants.ArmConstants.ARM_MOTOR_ID, MotorType.kBrushless);
    armPidController = new PIDController(Constants.ArmConstants.ARM_P, Constants.ArmConstants.ARM_I, Constants.ArmConstants.ARM_D);

    SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    armMotorConfig.smartCurrentLimit(Constants.ArmConstants.ARM_MAX_CURRENT);
    arm.configure(armMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    resetEncoders();
    this.armPidController.setTolerance(0.5);
}
    public void resetEncoders() {
        arm.getEncoder().setPosition(0);
        targetPosition = 0.0;
    }


    public double getCurrentPosition() {
        return arm.getEncoder().getPosition();
    }

    // Set the target position for the pivot to hold or move to
    public void setTargetPosition(double position) {
       targetPosition = position;
        manualOverride = false;
    }

    public void manualControl(double speed) {
        arm.set(0.2);
    }
    
    public boolean atSetpoint() {
        return armPidController.atSetpoint();
    }

    public void moveToSetpoint(double setpoint) {
        double output = armPidController.calculate(getCurrentPosition(), setpoint);
        output = MathUtil.clamp(output, -Constants.ArmConstants.ARM_MAX_SPEED,
                       Constants.ArmConstants.ARM_MAX_SPEED);
        arm.set(output);

    }

    
    public void endManualControl() {
        manualOverride = false;
    }


    @Override
    public void periodic(){
           if (manualOverride) {
            arm.set(manualSpeed);
            targetPosition = getCurrentPosition(); // Update target position to current if in manual mode
        } else {
            moveToSetpoint(targetPosition);
        }
        
    }

}