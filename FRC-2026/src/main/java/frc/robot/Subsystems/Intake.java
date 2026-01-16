package frc.robot.Subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

    public static Intake getInstance () {
        if (m_Instance == null)
            m_Instance = new Intake(Constants.Intake.intakeMotorID);
        return m_Instance;
    }

}
