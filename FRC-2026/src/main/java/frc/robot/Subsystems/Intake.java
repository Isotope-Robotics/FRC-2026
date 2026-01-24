package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public SparkMax intakeMotor; 
    public TalonFX elbowMotor;

    private static Intake m_Instance;

    private Intake (int intakeMotorID, int elbowMotorID) {

        intakeMotor = new SparkMax(intakeMotorID, MotorType.kBrushless);
        elbowMotor = new TalonFX(elbowMotorID);

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

    //Intake system elbow control

    public void contract() {
        elbowMotor.setPosition(Math.PI/2);
    }

    public void extend() {
        elbowMotor.setPosition(Math.PI/6);
    }


    public static Intake getInstance () {
        if (m_Instance == null)
            m_Instance = new Intake(Constants.Intake.intakeMotorID, Constants.Intake.elbowMotorID);
        return m_Instance;
    }
    


}
