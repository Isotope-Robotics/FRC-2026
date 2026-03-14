package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    // Base CANcoder config — MagnetOffset is set per-module in SwerveModule
    public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs() {

        /** Swerve CANCoder Base Configuration
         *  Only direction is set here. MagnetOffset is applied per-module
         *  in SwerveModule since each module has a unique offset value.
         */
        swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;

        /** Swerve Angle Motor Configuration
         *  Uses internal RotorSensor with SensorToMechanismRatio.
         *  The TalonFX position will read in wheel (mechanism) rotations.
         *  We seed the position from the CANcoder once at boot after
         *  applying MagnetOffset to the CANcoder.
         */
        swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        swerveAngleFXConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.angleGearRatio;

        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.angleCurrentLimit;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.angleCurrentThreshold;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.angleCurrentThresholdTime;

        swerveAngleFXConfig.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleFXConfig.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleFXConfig.Slot0.kD = Constants.Swerve.angleKD;

        /** Swerve Drive Motor Configuration */
        swerveDriveFXConfig.MotorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;

        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentThreshold;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;

        swerveDriveFXConfig.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveFXConfig.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveFXConfig.Slot0.kD = Constants.Swerve.driveKD;

        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
    }
}