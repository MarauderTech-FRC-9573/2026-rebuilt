package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualControl extends Command {
    private final ArmSubsystem arm;
    private final double speedSupplier;

    public ArmManualControl(ArmSubsystem armSubsystem, double speedSupplier) {
        this.arm = armSubsystem;
        this.speedSupplier = speedSupplier;
        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        arm.manualControl(speedSupplier);
    }
    
    @Override
    public void end(boolean interrupted) { 
        arm.endManualControl(); 
    }

    
}
