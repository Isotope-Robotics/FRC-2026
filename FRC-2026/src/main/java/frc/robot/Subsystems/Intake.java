package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public SparkMax intakeMotor; 
    public SparkMaxConfig intakeMotorConfig;

    public Intake(int IntakeMotorID) {
        intakeMotor = new SparkMax(IntakeMotorID, MotorType.kBrushless);
        intakeMotorConfig = new SparkMaxConfig();
        intakeMotor.configure(intakeMotorConfig);
    }
}
