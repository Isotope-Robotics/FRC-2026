package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AutoCommands.IntakeCommands;
import frc.robot.AutoCommands.ShooterCommands;

//Class for Auto Commands Only
public class RobotContainer {
    SendableChooser<Command> autoChooser;

    public RobotContainer(){

        // ADD AUTO COMMANDS HERE
        NamedCommands.registerCommand("Run Intake", IntakeCommands.runIntake());
        NamedCommands.registerCommand("Stop Intake", IntakeCommands.stopIntake());
        NamedCommands.registerCommand("Run Shooter", ShooterCommands.runShooter());
        NamedCommands.registerCommand("Stop Shooter", ShooterCommands.stopShooter());

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}
