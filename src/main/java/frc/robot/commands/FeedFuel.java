package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class FeedFuel extends Command{
    private FeederSubsystem feeder;
    private double speed;

    public FeedFuel(FeederSubsystem feeder2, double feederMotorSpeed) {
        //TODO Auto-generated constructor stub
    }

    public void Shoot(FeederSubsystem feeder, double speed){
        
        addRequirements(feeder);

        feeder = this.feeder;
        speed = this.speed;
    }

    @Override
    public void execute() {
        feeder.setFeeder(speed);
    }

    @Override
    public void end(boolean interrupted) {
        feeder.stop(); 
    }
}