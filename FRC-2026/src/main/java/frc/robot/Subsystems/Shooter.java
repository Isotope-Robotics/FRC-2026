package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    public SparkMax shooterMotor;
    public RelativeEncoder shooterEncoder;

    private Shooter (int shooterMotorID) {
        
        shooterMotor = new SparkMax(shooterMotorID, MotorType.kBrushless);
        shooterEncoder = shooterMotor.getEncoder();
        
        SparkMaxConfig shooterConfig = new SparkMaxConfig();
        shooterConfig.idleMode(IdleMode.kBrake);
        shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

}
