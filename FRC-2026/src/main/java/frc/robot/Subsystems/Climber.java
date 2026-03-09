package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public TalonFX rightClimberMotor, leftClimberMotor;

    private static Climber m_Instance;

    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private Climber (int rightClimberMotorID, int leftClimberMotorID) {

        rightClimberMotor = new TalonFX(rightClimberMotorID);
        leftClimberMotor = new TalonFX(leftClimberMotorID);

        TalonFXConfiguration rightClimberConfig = new TalonFXConfiguration();
        MotorOutputConfigs rightClimberOutput = rightClimberConfig.MotorOutput;
        rightClimberOutput.Inverted = Constants.Climber.rightClimberMotorInvert;
        rightClimberOutput.NeutralMode = Constants.Climber.rightClimberMotorIdleMode;
        rightClimberMotor.getConfigurator().apply(rightClimberConfig);

        TalonFXConfiguration leftClimberConfig = new TalonFXConfiguration();
        MotorOutputConfigs leftClimberOutput = leftClimberConfig.MotorOutput;
        leftClimberOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftClimberOutput.NeutralMode = NeutralModeValue.Brake;
        leftClimberMotor.getConfigurator().apply(leftClimberConfig);

            TalonFXConfiguration config = new TalonFXConfiguration();

            config.Slot0.kP = Constants.Climber.kP;
            config.Slot0.kI = Constants.Climber.kI;
            config.Slot0.kD = Constants.Climber.kD;

            rightClimberMotor.getConfigurator().apply(config);
            leftClimberMotor.getConfigurator().apply(config);
    }
       
    public void climbUp() {
        positionRequest.Position = Constants.Climber.CLIMB_UP_POSITION;
        rightClimberMotor.setControl(positionRequest);
        leftClimberMotor.setControl(positionRequest);
    }

    public void climbDown() {
        positionRequest.Position = Constants.Climber.CLIMB_DOWN_POSITION;
        rightClimberMotor.setControl(positionRequest);
        leftClimberMotor.setControl(positionRequest);
    }

    public static Climber getInstance () {
        if (m_Instance == null)
            m_Instance = new Climber(Constants.Climber.rightClimberMotorID, Constants.Climber.leftClimberMotorID);
        return m_Instance;
    }
}
