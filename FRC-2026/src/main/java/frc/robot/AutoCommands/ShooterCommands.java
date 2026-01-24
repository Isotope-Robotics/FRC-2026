package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter;

public class ShooterCommands {

    public static Shooter shooter = Shooter.getInstance();

    public static Command runShooter(){
        return Commands.runOnce(() ->{
            shooter.shoot(Constants.Shooter.maxVelocity);
        });
    }

    public static Command stopShooter(){
        return Commands.runOnce(() ->{
            shooter.stop();
        });
    }
    
}
