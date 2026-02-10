package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmSetpointControl extends Command {
    private final ArmSubsystem arm;
    private final double setpoint;

    public ArmSetpointControl(ArmSubsystem arm, double setpoint) {
        this.arm = arm;
        this.setpoint = setpoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setTargetPosition(setpoint);
    }

    @Override
    public boolean isFinished() {
        return arm.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // No action needed; subsystem will continue to hold at targetPosition
    }
}