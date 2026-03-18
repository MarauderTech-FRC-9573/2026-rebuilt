package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootFuel extends Command{
    private ShooterSubsystem shooter;
    private double speed;

    public ShootFuel(ShooterSubsystem shooter2, double shooterMotorSpeed) {
        //TODO Auto-generated constructor stub
    }

    public void Shoot(ShooterSubsystem shooter, double speed){
        addRequirements(shooter);

        shooter = this.shooter;
        speed = this.speed;
        }

    @Override
    public void execute() {
        shooter.setShooter(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop(); // Stop the shooter when the command ends
    }
}
