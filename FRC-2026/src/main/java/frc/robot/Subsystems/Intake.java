package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public SparkMax intakeMotor; 

    private static Intake m_Instance;

    private Intake (int intakeMotorID) {

        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(Constants.Intake.intakeMotorIdleMode);
        intakeConfig.inverted(Constants.Intake.intakeMotorInvert);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void set(double value) {
        intakeMotor.set(value);
    }

    public void stop() {
        intakeMotor.set(0);
    }

    //May need to switch the signs on intake and outtake
    public void intake() {
        intakeMotor.set(-1);
    }

    public void outtake() {
        intakeMotor.set(1);
    }

    public static Intake getInstance () {
        if (m_Instance == null)
            m_Instance = new Intake(Constants.Intake.intakeMotorID);
        return m_Instance;
    }
    


}
