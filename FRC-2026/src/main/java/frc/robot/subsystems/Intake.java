package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {

    public SparkFlex intakeMotor; 
    public TalonFX elbowMotor;

    private static Intake m_Instance;

    public enum IntakeState {
        ON,
        OFF
    }

    public IntakeState state = IntakeState.OFF;

    private Intake (int intakeMotorID, int elbowMotorID) {

        intakeMotor = new SparkFlex(intakeMotorID, MotorType.kBrushless);

        SparkFlexConfig intakeConfig = new SparkFlexConfig();
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
        intakeMotor.set(-Constants.Intake.intakeVelocity);
    }

    public void outtake() {
        intakeMotor.set(Constants.Intake.intakeVelocity);
    }

    //Intake system elbow control

    public void contract() {
        elbowMotor.setPosition(0);
    }

    public void extend() {
        elbowMotor.setPosition(5/16);
    }


    public static Intake getInstance () {
        if (m_Instance == null)
            m_Instance = new Intake(Constants.Intake.intakeMotorID, Constants.Intake.elbowMotorID);
        return m_Instance;
    }
    


}
