package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    public TalonFX shooterMotor;
    public SparkFlex feederMotor;
    public SparkFlex spindexerMotor;
    public RelativeEncoder spindexerEncoder;
    
    private static Shooter m_Instance = null;

    private Shooter (int shooterMotorID, int feederMotorID, int spindexerMotorID) {

        shooterMotor = new TalonFX(shooterMotorID);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        MotorOutputConfigs shooterOutput = shooterConfig.MotorOutput;
        shooterOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterOutput.NeutralMode = NeutralModeValue.Coast;
        shooterMotor.getConfigurator().apply(shooterConfig);
        
        feederMotor = new SparkFlex(feederMotorID, MotorType.kBrushless);
                
        SparkFlexConfig feederConfig = new SparkFlexConfig();
        feederConfig.idleMode(Constants.Shooter.feederMotorIdleMode);
        feederConfig.inverted(Constants.Shooter.feederMotorInvert);
        feederMotor.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        spindexerMotor = new SparkFlex(spindexerMotorID, MotorType.kBrushless);
        spindexerEncoder = spindexerMotor.getEncoder();
        
        SparkFlexConfig spindexerConfig = new SparkFlexConfig();
        spindexerConfig.idleMode(Constants.Shooter.spindexerMotorIdleMode);
        spindexerConfig.inverted(Constants.Shooter.spindexerMotorInvert);
        spindexerMotor.configure(spindexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void shoot(int velocity){
        shooterMotor.set(Constants.Shooter.shooterPID.calculate(velocity - shooterMotor.getVelocity().getValueAsDouble()));
    }

    public void stop(){
        shooterMotor.set(0);
    }
        
    public void startFeeder(){
        feederMotor.set(1);
    }

    public void stopFeeder(){
        feederMotor.set(0);
    }

    public void spindex(){
        spindexerMotor.set(Constants.Shooter.spindexerPID.calculate(Constants.Shooter.spindexerVelocity - spindexerEncoder.getVelocity()));
    }

    public void stopSpindex(){
        spindexerMotor.set(0);
    }



    public static Shooter getInstance () {
        if (m_Instance == null)
            m_Instance = new Shooter(Constants.Shooter.shooterMotorID, Constants.Shooter.feederMotorID, Constants.Shooter.spindexerMotorID);
        return m_Instance;
    }

}
