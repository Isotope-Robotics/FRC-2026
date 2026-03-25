package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import frc.robot.lib.math.Conversions;
import frc.robot.lib.util.SwerveModuleConstants;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    private TalonFX mDriveMotor;
    private TalonFX mAngleMotor;
    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(
            Constants.Swerve.driveKS,
            Constants.Swerve.driveKV,
            Constants.Swerve.driveKA);

    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;

        // ── CANcoder ────────────────────────────────────────────────────────
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.SensorDirection = Constants.Swerve.cancoderInvert;
        cancoderConfig.MagnetSensor.MagnetOffset = -angleOffset.getRotations();
        applyUntilOk(5, () -> angleEncoder.getConfigurator().apply(cancoderConfig, 0.25));

        // ── Angle Motor ──────────────────────────────────────────────────────
        mAngleMotor = new TalonFX(moduleConstants.angleMotorId);
        applyUntilOk(5, () -> mAngleMotor.getConfigurator().apply(
                Robot.ctreConfigs.swerveAngleFXConfig, 0.25));
        resetToAbsolute();

        // ── Drive Motor ──────────────────────────────────────────────────────
        // Build a per-module drive config so each module can have its own
        // invert. We copy the shared base config then override InvertedValue
        // with the value stored in this module's SwerveModuleConstants.
        mDriveMotor = new TalonFX(moduleConstants.driveMotorId);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.Inverted = moduleConstants.driveMotorInvert; // per-module
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Feedback.SensorToMechanismRatio = Constants.Swerve.driveGearRatio;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        driveConfig.CurrentLimits.StatorCurrentLimit = Constants.Swerve.driveCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.Swerve.driveCurrentLimit;
        driveConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Swerve.driveCurrentThreshold;
        driveConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Swerve.driveCurrentThresholdTime;
        driveConfig.Slot0.kP = Constants.Swerve.driveKP;
        driveConfig.Slot0.kI = Constants.Swerve.driveKI;
        driveConfig.Slot0.kD = Constants.Swerve.driveKD;
        driveConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        driveConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.Swerve.openLoopRamp;
        driveConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        driveConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.Swerve.closedLoopRamp;
        applyUntilOk(5, () -> mDriveMotor.getConfigurator().apply(driveConfig, 0.25));
        applyUntilOk(5, () -> mDriveMotor.getConfigurator().setPosition(0.0, 0.25));
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState.optimize(getState().angle);
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if (isOpenLoop) {
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        } else {
            driveVelocity.Velocity = Conversions.MPSToRPS(
                    desiredState.speedMetersPerSecond,
                    Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
    }

    public Rotation2d getCANCoder() {
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute() {
        StatusSignal<Angle> absSignal = angleEncoder.getAbsolutePosition();
        absSignal.waitForUpdate(0.25);
        double absolutePositionRotations = absSignal.getValueAsDouble();
        applyUntilOk(10, () -> mAngleMotor.setPosition(absolutePositionRotations, 0.25));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                Conversions.RPSToMPS(
                        mDriveMotor.getVelocity().getValueAsDouble(),
                        Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                Conversions.rotationsToMeters(
                        mDriveMotor.getPosition().getValueAsDouble(),
                        Constants.Swerve.wheelCircumference),
                Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble()));
    }

    public double getDriveCurrent() {
        return mDriveMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getAngleCurrent() {
        return mAngleMotor.getSupplyCurrent().getValueAsDouble();
    }

    private void applyUntilOk(int maxAttempts, java.util.function.Supplier<StatusCode> call) {
        for (int i = 0; i < maxAttempts; i++) {
            StatusCode code = call.get();
            if (code.isOK()) break;
        }
    }
}