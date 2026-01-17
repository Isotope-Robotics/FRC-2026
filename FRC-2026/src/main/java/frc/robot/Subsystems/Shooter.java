package frc.robot.Subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    public SparkMax shooterMotor;
    public RelativeEncoder shooterEncoder;

    private static Shooter m_Instance = null;

    private Shooter (int shooterMotorID) {

        shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();
        
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(Constants.Shooter.shooterMotorIdleMode);
        shooterConfig.inverted(Constants.Shooter.shooterMotorInvert);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public void shoot(int velocity){
        shooterMotor.set(Constants.Shooter.shooterPID.calculate(velocity - shooterEncoder.getVelocity()));
    }

    public void stop(){
        shooterMotor.set(0);
    }

    public static Shooter getInstance () {
        if (m_Instance == null)
            m_Instance = new Shooter(Constants.Shooter.shooterMotorID);
        return m_Instance;
    }

}
