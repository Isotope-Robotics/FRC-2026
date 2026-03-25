package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    public TalonFX shooterMotor;
    public TalonFX shooter2Motor;
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
    public TalonFX hoodMotor;
    public SparkFlex feederMotor;
    public SparkFlex spindexerMotor;
    public TalonFX turretMotor;
    public RelativeEncoder spindexerEncoder;
    // Proximity sensor
    // Initialize a DigitalInput on DIO port 0
    public DigitalInput proxSensor;
    
    private static Shooter m_Instance = null;

    public enum ShooterState {
        ON,
        OFF
    }

    public ShooterState state = ShooterState.OFF;

    private Shooter (int shooterMotorID, int shooter2MotorID, int hoodMotorID, int feederMotorID, int spindexerMotorID, int turretMotorID) {

        shooterMotor = new TalonFX(shooterMotorID);

        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        MotorOutputConfigs shooterOutput = shooterConfig.MotorOutput;
        shooterOutput.Inverted = Constants.Shooter.shooterMotorInvert;
        shooterOutput.NeutralMode = Constants.Shooter.shooterMotorIdleMode;
        Slot0Configs shooterSlot0 = shooterConfig.Slot0;
        shooterSlot0.kP = Constants.Shooter.shooterKP;
        shooterSlot0.kI = Constants.Shooter.shooterKI;
        shooterSlot0.kD = Constants.Shooter.shooterKD;
        shooterSlot0.kV = Constants.Shooter.shooterKV;
        shooterSlot0.kS = Constants.Shooter.shooterKS;
        shooterMotor.getConfigurator().apply(shooterConfig);

        shooter2Motor = new TalonFX(shooter2MotorID);

        TalonFXConfiguration shooter2Config = new TalonFXConfiguration();
        MotorOutputConfigs shooter2Output = shooter2Config.MotorOutput;
        shooter2Output.Inverted = Constants.Shooter.shooter2MotorInvert;
        shooter2Output.NeutralMode = Constants.Shooter.shooter2MotorIdleMode;
        Slot0Configs shooter2Slot0 = shooter2Config.Slot0;
        shooter2Slot0.kP = Constants.Shooter.shooterKP;
        shooter2Slot0.kI = Constants.Shooter.shooterKI;
        shooter2Slot0.kD = Constants.Shooter.shooterKD;
        shooter2Slot0.kV = Constants.Shooter.shooterKV;
        shooter2Slot0.kS = Constants.Shooter.shooterKS;
        shooter2Motor.getConfigurator().apply(shooter2Config);

        hoodMotor = new TalonFX(hoodMotorID);

        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        MotorOutputConfigs hoodOutput = hoodConfig.MotorOutput;
        hoodOutput.Inverted = Constants.Shooter.hoodMotorInvert;
        hoodOutput.NeutralMode = Constants.Shooter.hoodMotorIdleMode;
        hoodMotor.getConfigurator().apply(hoodConfig);
        
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

        turretMotor = new TalonFX(turretMotorID);

        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        MotorOutputConfigs turretOutput = hoodConfig.MotorOutput;
        turretOutput.Inverted = Constants.Shooter.turretMotorInvert;
        turretOutput.NeutralMode = Constants.Shooter.turretMotorIdleMode;
        turretMotor.getConfigurator().apply(turretConfig);

        proxSensor = new DigitalInput(0);
        
    }

    public void turretClockwise () {
        
        turretMotor.set(turretMotor.getPosition().getValueAsDouble() + 1.0/40.0);

    }

    public void turretCounterclockwise () {

        turretMotor.set(turretMotor.getPosition().getValueAsDouble() - 1.0/40.0);

    }

    public void turretStop () {
        
        turretMotor.set(0);

    }

    /** Set both shooter motors to a target RPS using onboard velocity PID. */
    public void setLauncherRPS(double rps) {
        shooterMotor.setControl(m_velocityRequest.withVelocity(rps));
        shooter2Motor.setControl(m_velocityRequest.withVelocity(rps));
    }

    public double getShooterRPS() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public void startLaunchers(){
        setLauncherRPS(Constants.Shooter.highShootRPS);
    }

    public void stopLaunchers(){
        shooterMotor.set(0);
        shooter2Motor.set(0);
    }
        
    public void startFeeder(){
        feederMotor.set(Constants.Shooter.feederVelocity);
    }

    public void stopFeeder(){
        feederMotor.set(0);
    }

    public int fuelCheck() {
        //! used here since a true value for prox sensor would mean sensor is open
        boolean fuelDetected = !proxSensor.get();
            if (fuelDetected) {
                SmartDashboard.putBoolean("Fuel Present", fuelDetected);
                return 1;
            }
            return 0;
    }

    public void spindex(){
        spindexerMotor.set(Constants.Shooter.spindexerVelocity);
        /*if (fuelCheck()!=1){
            spindexerMotor.set(Constants.Shooter.spindexerPID.calculate(spindexerEncoder.getVelocity(), Constants.Shooter.spindexerVelocity));
        }
        else {
            spindexerMotor.set(0);
        }*/
    }

    public void stopSpindex(){
        spindexerMotor.set(0);
    }

    public static Shooter getInstance () {
        if (m_Instance == null)
            m_Instance = new Shooter(Constants.Shooter.shooterMotorID, Constants.Shooter.shooter2MotorID, Constants.Shooter.hoodMotorID, Constants.Shooter.feederMotorID, Constants.Shooter.spindexerMotorID, Constants.Shooter.turretMotorID);
        return m_Instance;
    }

}
