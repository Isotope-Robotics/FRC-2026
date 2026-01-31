package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

    public SparkMax rightClimberMotor;
    public SparkMax leftClimberMotor;
    public RelativeEncoder rightClimberEncoder;
    public RelativeEncoder leftClimberEncoder;
    
    private static Climber m_Instance = null;

    private Climber (int rightClimberMotorID, int leftClimberMotorID) {
        rightClimberMotor = new SparkMax(rightClimberMotorID, MotorType.kBrushless);
        rightClimberEncoder = rightClimberMotor.getEncoder();
        leftClimberMotor = new SparkMax(leftClimberMotorID, MotorType.kBrushless);
        leftClimberEncoder = leftClimberMotor.getEncoder();

        SparkMaxConfig rightClimberConfig = new SparkMaxConfig();
        rightClimberConfig.idleMode(Constants.Climber.rightClimberMotorIdleMode);
        rightClimberConfig.inverted(Constants.Climber.rightClimberMotorInvert);
        rightClimberMotor.configure(rightClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig leftClimberConfig = new SparkMaxConfig();
        leftClimberConfig.idleMode(Constants.Climber.leftClimberMotorIdleMode);
        leftClimberConfig.inverted(Constants.Climber.leftClimberMotorInvert);
        leftClimberMotor.configure(leftClimberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void stop() {
        rightClimberMotor.set(0);
        leftClimberMotor.set(0);
    }

    


    public static Climber getInstance () {
        if (m_Instance == null)
            m_Instance = new Climber(Constants.Climber.rightClimberMotorID, Constants.Climber.leftClimberMotorID);
        return m_Instance;
    }
}
