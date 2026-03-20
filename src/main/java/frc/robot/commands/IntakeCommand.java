package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{

    private IntakeSubsystem intake;
    private double speed; 

    public IntakeCommand(IntakeSubsystem intake, double speed) {
         this.intake = intake;
        this.speed = speed;
    }

    @Override
    public void execute() {
       intake.setIntake(speed);
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
