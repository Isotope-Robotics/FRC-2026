package frc.robot.AutoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Intake;

public class IntakeCommands {

    public static Intake intake = Intake.getInstance();

    public static Command runIntake () {
        return Commands.runOnce(() -> {
            intake.intake();
        });
    }

    public static Command stopIntake () {
        return Commands.runOnce(() -> {
            intake.stop();
        });
    }
}
