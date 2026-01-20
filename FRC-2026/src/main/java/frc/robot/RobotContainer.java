package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

//Class for Auto Commands Only
public class RobotContainer {
    SendableChooser<Command> autoChooser;

    public RobotContainer(){

        // ADD AUTO COMMANDS HERE
        // NamedCommands.registerCommand("DropIntake", dropToPickUp);
        // NamedCommands.registerCommand("UpIntake", upIntake);
        // NamedCommands.registerCommand("IntakeNote", IntakeCommands.IntakeNote());
        // NamedCommands.registerCommand("Shoot", ShooterCommands.shoot());
        // NamedCommands.registerCommand("IntakeStop", IntakeCommands.stopIntake());
        // NamedCommands.registerCommand("IntakeLess", IntakeCommands.IntakeLess());
        // NamedCommands.registerCommand("StopWrist", IntakeCommands.StopWrist());
        // NamedCommands.registerCommand("NoteAutoAim", SwerveCommands.NoteAutoAim());
        // NamedCommands.registerCommand("LessNoteAutoAim", SwerveCommands.LessNoteAutoAim());
        // NamedCommands.registerCommand("Shoot2", ShooterCommands.shoot2());

        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        SmartDashboard.putData("Auto Mode", autoChooser);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
    
}
