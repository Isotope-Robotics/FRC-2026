package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.lib.input.HIDEventHandler;
import frc.robot.lib.util.FalconSwerveConstants;
import frc.robot.lib.util.SwerveModuleConstants;

public class Constants {

        public static final class Controllers {
                public static final Joystick driver1 = new Joystick(0);
                public static final XboxController driver2 = new XboxController(1);
                public static final HIDEventHandler driver2Handler = new HIDEventHandler(driver2);
                public static final double stickDeadband = 0.3;
        }

        // Swerve Module Constants Class
        public static final class Swerve {

                // Pigeon CAN ID
                public static final int pigeonId = 8;

                // Module Drive Ratios
                public static final FalconSwerveConstants chosenModule = FalconSwerveConstants.SDS.MK4i
                                .Falcon500(FalconSwerveConstants.SDS.MK4i.driveRatios.L3);

                public static final FalconSwerveConstants module0 = FalconSwerveConstants.SDS.MK4i
                                .Falcon500Inverted(FalconSwerveConstants.SDS.MK4i.driveRatios.L3);

                // Drivetrain Constants
                // Wheel positions: 11 inches from center on each axis (from TunerConstants XPos/YPos)
                public static final double trackWidth = Units.inchesToMeters(22.0);
                public static final double wheelBase = Units.inchesToMeters(22.0);
                public static final double wheelCircumference = chosenModule.wheelCircumference;

                public static final Translation2d flModuleOffset = new Translation2d(wheelBase / 2.0, trackWidth / 2.0);

                // Swerve Kinematics
                // Modules numbered counter-clockwise starting from Front Left (Mod0)
                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),   // mod0 front left // MOD0 is front left good
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),  // mod1 back left // MOD1 is back left good
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0), // mod2 back right // MOD2 is backk right
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0)); // mod3 front right // MOd3 Is front right good

                // Gear Ratios (from TunerConstants kDriveGearRatio / kSteerGearRatio)
                public static final double driveGearRatio = 5.2734375;
                public static final double angleGearRatio = 26.09090909090909;

                // Motor Inverts
                public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
                // Left side not inverted (kInvertLeftSide = false), right side inverted (kInvertRightSide = true)
                public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

                // Angle Encoder Invert
                public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

                // Swerve Current Limiting
                // Steer motor stator current limit from TunerConstants steerInitialConfigs
                public static final int angleCurrentLimit = 60;
                public static final int angleCurrentThreshold = 40;
                public static final double angleCurrentThresholdTime = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                // Drive slip current from TunerConstants kSlipCurrent
                public static final int driveCurrentLimit = 120;
                public static final int driveCurrentThreshold = 120;
                public static final double driveCurrentThresholdTime = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                /*
                        * These values are used by the drive falcon to ramp in open loop and closed
                        * loop driving.
                        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
                        */
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                // Angle Motor PID Values (from TunerConstants steerGains)
                public static final double angleKP = 100.0;
                public static final double angleKI = 0.0;
                public static final double angleKD = 0.5;

                // Drive Motor PID Values (from TunerConstants driveGains)
                public static final double driveKP = 0.1;
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKF = 0.0;

                public static final double driveKS = 0.0;
                public static final double driveKV = 0.124;
                public static final double driveKA = 0.0;

                // Swerve Profiling Values
                // Meters per Second (from TunerConstants kSpeedAt12Volts)
                public static final double maxSpeed = 5.85;
                // Radians per Second
                public static final double driveRadius = Math.hypot(wheelBase, trackWidth) / 2.0;
                public static final double maxAngularVelocity = maxSpeed / driveRadius;

                // Neutral Modes
                public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
                public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

                // Module Specific Constants
                // Modules numbered counter-clockwise starting at Front Left
                // Encoder offsets converted from TunerConstants rotations to degrees (* 360)


                /* Front Left Module - Module 0 — MK5n Layout A */
                public static final class Mod0 {
                public static final int driveMotorID = 0;
                public static final int angleMotorID = 4;
                public static final int canCoderID = 54;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-6.240234);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                        driveMotorID, angleMotorID, canCoderID, angleOffset,
                        InvertedValue.CounterClockwise_Positive); // Layout A
                }

                /* Back Left Module - Module 1 — MK5n Layout B */
                public static final class Mod1 {
                public static final int driveMotorID = 1;
                public static final int angleMotorID = 5;
                public static final int canCoderID = 55;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(95.185547);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                        driveMotorID, angleMotorID, canCoderID, angleOffset,
                        InvertedValue.CounterClockwise_Positive); // Layout B — mirrored
                }

                /* Back Right Module - Module 2 — MK5n Layout A */
                public static final class Mod2 {
                public static final int driveMotorID = 2;
                public static final int angleMotorID = 6;
                public static final int canCoderID = 56;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(130.166016);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                        driveMotorID, angleMotorID, canCoderID, angleOffset,
                        InvertedValue.Clockwise_Positive); // Layout A
                }

                /* Front Right Module - Module 3 — MK5n Layout B */
                public static final class Mod3 {
                public static final int driveMotorID = 3;
                public static final int angleMotorID = 7;
                public static final int canCoderID = 57;
                public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-142.822266);
                public static final SwerveModuleConstants constants = new SwerveModuleConstants(
                        driveMotorID, angleMotorID, canCoderID, angleOffset,
                        InvertedValue.Clockwise_Positive); // Layout B — mirrored
                }

        }
        
        public static final class Shooter {
                
                // TODO: REMOVE DUMMY CONSTANTS
                public static final int shooterMotorID = 30;
                public static final InvertedValue shooterMotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue shooterMotorIdleMode = NeutralModeValue.Coast;

                public static final int shooter2MotorID = 31;
                public static final InvertedValue shooter2MotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue shooter2MotorIdleMode = NeutralModeValue.Coast;
                // Onboard velocity PID gains for shooter TalonFX motors (Phoenix 6 Slot 0)
                // kV: volts per RPS — approx 12V / free_speed_RPS (tune this first)
                // kP: volts per RPS of error (tune after kV)
                public static final double shooterKP = 0.1;   // TODO: tune
                public static final double shooterKI = 0.0;
                public static final double shooterKD = 0.0;
                public static final double shooterKV = 0.12;  // TODO: tune (12V / ~100 RPS free speed)
                public static final double shooterKS = 0.0;   // TODO: tune (static friction volts)

                // RPS setpoints (rotations per second)
                public static final double closeShootRPS    = 6.0;
                public static final double cornerShootRPS   = 6.2;
                public static final double highShootRPS     = 7.5;
                
                public static final int hoodMotorID = 33;
                public static final InvertedValue hoodMotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue hoodMotorIdleMode = NeutralModeValue.Brake;

                public static final int turretMotorID = 32;
                public static final InvertedValue turretMotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue turretMotorIdleMode = NeutralModeValue.Brake;

                public static final int feederMotorID = 21;
                public static final boolean feederMotorInvert = false;
                public static final IdleMode feederMotorIdleMode = IdleMode.kBrake;
                public static final double feederVelocity = 1.000;

                public static final int spindexerMotorID = 20;
                public static final boolean spindexerMotorInvert = false;
                public static final IdleMode spindexerMotorIdleMode = IdleMode.kBrake;
                public static final double spindexerVelocity = 0.8;
                public static final PIDController spindexerPID = new PIDController(
                        0.1,
                        0,
                        0
                );

                public static final double lowPower = 0.300;
                public static final double mediumPower = 0.450;
                public static final double highPower = 0.600;
                public static final double maxPower = 0.750;
                public static final double concussionPower = 1.000;

                public static final double hubWidth = 47.0;

       }

        public static final class Vision {
                public static final String limelightName = "april";
        }

        public static final class Intake {
                // TODO: REMOVE DUMMY CONSTANTS
                public static final int intakeMotorID = 10;
                public static final boolean intakeMotorInvert = true;
                public static final IdleMode intakeMotorIdleMode = IdleMode.kBrake;
                public static final int elbowMotorID = 11;
                public static final double intakeVelocity = 0.200;
        }
        public static final class Climber {
                public static final int rightClimberMotorID = 40;
                public static final InvertedValue rightClimberMotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue rightClimberMotorIdleMode = NeutralModeValue.Brake;
                public static final int leftClimberMotorID = 41;
                public static final boolean leftClimberMotorInvert = true;
                public static final IdleMode leftClimberMotorIdleMode = IdleMode.kBrake;

                //PID with solt 0, whatever that means
                public static final double kP = 60.0;
                public static final double kI = 0.0;
                public static final double kD = 2.0;

                // Positions (rotations)
                public static final double CLIMB_UP_POSITION = 25.0;
                public static final double CLIMB_DOWN_POSITION = 0.0;

                // Solenoid ports on REV Pneumatics Hub
                public static final int solenoidForwardChannel = 1;
                public static final int solenoidReverseChannel = 2;
        }
}
