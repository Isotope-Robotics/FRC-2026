package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

        SparkMaxConfig intakeConfig = new SparkMaxConfig();
        intakeConfig.idleMode(Constants.Intake.intakeMotorIdleMode);
        intakeConfig.inverted(Constants.Intake.intakeMotorInvert);
        intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elbowMotor = new TalonFX(elbowMotorID);

        TalonFXConfiguration elbowConfig = new TalonFXConfiguration();
        MotorOutputConfigs elbowOutput = elbowConfig.MotorOutput;
        elbowOutput.Inverted = InvertedValue.Clockwise_Positive;
        elbowOutput.NeutralMode = NeutralModeValue.Brake;
        elbowMotor.getConfigurator().apply(elbowConfig);
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
        PositionVoltage request = new PositionVoltage(1/4);
        elbowMotor.setControl(request);
    }

    public void extend() {
        PositionVoltage request = new PositionVoltage(1/12);
        elbowMotor.setControl(request);
    }


    public static Intake getInstance () {
        if (m_Instance == null)
            m_Instance = new Intake(Constants.Intake.intakeMotorID, Constants.Intake.elbowMotorID);
        return m_Instance;
    }
    


}
