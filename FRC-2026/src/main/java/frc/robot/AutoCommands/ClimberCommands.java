package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Climber;

public class ClimberCommands {
    
    public static Climber climber = Climber.getInstance();

    public static Command climbUp () {
        return Commands.runOnce(() -> {
            climber.climbUp();
        });
    }

    public static Command climbDown () {
        return Commands.runOnce(() -> {
            climber.climbDown();
        });
    }

}
