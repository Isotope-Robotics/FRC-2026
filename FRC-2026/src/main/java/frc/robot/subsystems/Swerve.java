package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public Field2d field = new Field2d();

    private static Swerve m_Instance = null;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonId);
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Constants.Swerve.Mod0.constants),
                new SwerveModule(1, Constants.Swerve.Mod1.constants),
                new SwerveModule(2, Constants.Swerve.Mod2.constants),
                new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        // Phoenix 6 Pigeon2 getYaw() StatusSignal is CCW-positive — this
        // already matches the WPILib convention, so no negation is needed.
        swerveOdometry = new SwerveDriveOdometry(
                Constants.Swerve.swerveKinematics,
                getGyroYaw(),
                getModulePositions());

        // PathPlanner config
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        if (config != null) {
            AutoBuilder.configure(
                    this::getPose,
                    this::setPose,
                    this::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> driveRobotRelative(speeds),
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0),
                            new PIDConstants(5.0, 0.0, 0.0)),
                    config,
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    this);
        } else {
            System.err.println("PathPlanner RobotConfig failed to load — AutoBuilder not configured!");
        }

        PathPlannerLogging.setLogActivePathCallback(
                (poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
    }

    public void drive(Translation2d translation, double rotation, boolean isFieldRel, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                isFieldRel
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation,
                                getHeading())
                        : new ChassisSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation));

       // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
       // SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading) {
        swerveOdometry.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading() {
        swerveOdometry.resetPosition(
                getGyroYaw(),
                getModulePositions(),
                new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    /**
     * Returns the robot heading as a Rotation2d.
     * Phoenix 6 Pigeon2 getYaw() is CCW-positive, matching WPILib convention.
     * Use this everywhere — do NOT negate.
     */
    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    /** Raw yaw in degrees for telemetry display only. */
    public double getRawYaw() {
        return gyro.getYaw().getValueAsDouble();
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }

    public void resetModulesToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    /** X-pattern wheel lock. */
    private final Rotation2d lockAngleA = Rotation2d.fromDegrees(45);
    private final Rotation2d lockAngleB = Rotation2d.fromDegrees(-45);
    private final SwerveModuleState lockStateA = new SwerveModuleState(0.0, lockAngleA);
    private final SwerveModuleState lockStateB = new SwerveModuleState(0.0, lockAngleB);

    public void lock() {
        for (SwerveModule mod : mSwerveMods) {
            // Modules 1 & 3 get one diagonal, 0 & 2 get the other
            if (mod.moduleNumber == 1 || mod.moduleNumber == 3) {
                mod.setDesiredState(lockStateA, false);
            } else {
                mod.setDesiredState(lockStateB, false);
            }
        }
    }

    public static Swerve getInstance() {
        if (m_Instance == null) {
            m_Instance = new Swerve();
        }
        return m_Instance;
    }

    @Override
    public void periodic() {
        // Single authoritative odometry update — remove the extra calls
        // in Robot.java's teleopPeriodic() and autonomousPeriodic()
        swerveOdometry.update(getGyroYaw(), getModulePositions());
        field.setRobotPose(getPose());

        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder",
                    mod.getCANCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle",
                    mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity",
                    mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current",
                    mod.getDriveCurrent());
            // Fixed: was incorrectly calling getDriveCurrent() for angle current
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current",
                    mod.getAngleCurrent());
        }
    }

    // Kept for compatibility with Robot.java calls — delegates to periodic telemetry
    public void swerveCurrents() {
        for (SwerveModule mod : mSwerveMods) {
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Drive Current",
                    mod.getDriveCurrent());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle Current",
                    mod.getAngleCurrent());
        }
    }
}