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

    private Climber (int rightClimberMotorID, int leftClimberMotorID) {

        rightClimberMotor = new TalonFX(rightClimberMotorID);
        leftClimberMotor = new TalonFX(leftClimberMotorID);

        TalonFXConfiguration rightClimberConfig = new TalonFXConfiguration();
        MotorOutputConfigs rightClimberOutput = rightClimberConfig.MotorOutput;
        rightClimberOutput.Inverted = InvertedValue.Clockwise_Positive;
        rightClimberOutput.NeutralMode = NeutralModeValue.Brake;
        rightClimberMotor.getConfigurator().apply(rightClimberConfig);

        TalonFXConfiguration leftClimberConfig = new TalonFXConfiguration();
        MotorOutputConfigs leftClimberOutput = leftClimberConfig.MotorOutput;
        leftClimberOutput.Inverted = InvertedValue.Clockwise_Positive;
        leftClimberOutput.NeutralMode = NeutralModeValue.Brake;
        leftClimberMotor.getConfigurator().apply(leftClimberConfig);
    }

    public void stop() {
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
    }
    
    public void climb() {
        //Create the request (set target to 10 rotations)
        //NEED TO REFINE ROTATIONS
        var request = new PositionVoltage(10.0);
        rightClimberMotor.setControl(request);
        leftClimberMotor.setControl(request);
    }

    public void climbDown() {
        //Create the request (set target to -10 rotations) 
        //NEED TO REFINE ROTATIONS
        var request = new PositionVoltage(-10.0);
        rightClimberMotor.setControl(request);
        leftClimberMotor.setControl(request);
    }

    public static Climber getInstance () {
        if (m_Instance == null)
            m_Instance = new Climber(Constants.Climber.rightClimberMotorID, Constants.Climber.leftClimberMotorID);
        return m_Instance;
    }
}
