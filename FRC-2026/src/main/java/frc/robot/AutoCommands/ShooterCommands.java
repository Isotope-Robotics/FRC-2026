package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter;

public class ShooterCommands {

    public static Shooter shooter = Shooter.getInstance();

    public static Command runShooter () {
        return Commands.runOnce(() -> {
            shooter.startLaunchers();
        }).andThen(() -> {
            shooter.startFeeder();
        }).andThen(() -> {
            shooter.spindex();
        });
    }

    public static Command stopShooter () {
        return Commands.runOnce(() -> {
            shooter.stopSpindex();
        }).andThen(() -> {
            shooter.stopFeeder();
        }).andThen(() -> {
            shooter.stopLaunchers();
        });
    }
    
}
