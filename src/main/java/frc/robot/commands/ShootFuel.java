package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootFuel extends Command{
    private ShooterSubsystem shooter;
    private double speed;

    public ShootFuel(ShooterSubsystem shooter, double speed) {
        this.shooter = shooter;
        this.speed = speed;  
        addRequirements(shooter);  
    }

    @Override
    public void execute() {
        shooter.run(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop(); // Stop the shooter when the command ends
    }
}
