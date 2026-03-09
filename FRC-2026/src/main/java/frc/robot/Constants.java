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
import frc.robot.Lib.Util.FalconSwerveConstants;
import frc.robot.Lib.Util.SwerveModuleConstants;

public class Constants {

        public static final class Controllers {
                public static final Joystick driver1 = new Joystick(0);
                public static final XboxController driver2 = new XboxController(1);
                public static final double stickDeadband = 0.3;
        }

        // Swerve Module Constants Class
        public static final class Swerve {

                // Pigeon CAN ID
                public static final int pigeonId = 13;

                // Module Drive Ratios
                public static final FalconSwerveConstants chosenModule = FalconSwerveConstants.SDS.MK4i
                                .Falcon500(FalconSwerveConstants.SDS.MK4i.driveRatios.L69);

                public static final FalconSwerveConstants module0 = FalconSwerveConstants.SDS.MK4i
                                .Falcon500Inverted(FalconSwerveConstants.SDS.MK4i.driveRatios.L69);

                // Drivetrain Constants (11in module offsets from Tuner X)
                public static final double trackWidth = Units.inchesToMeters(22.0);
                public static final double wheelBase = Units.inchesToMeters(22.0);

                // 2 inch radius wheel
                public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

                public static final Translation2d flModuleOffset =
                        new Translation2d(wheelBase / 2.0, trackWidth / 2.0);

                // Kinematics
                public static final SwerveDriveKinematics swerveKinematics =
                        new SwerveDriveKinematics(
                                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

                // Gear Ratios
                public static final double driveGearRatio = 5.2734375;
                public static final double angleGearRatio = 26.09090909090909;

                // Motor Inverts
                public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
                public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

                // Encoder invert
                public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

                // Current Limits
                public static final int angleCurrentLimit = 60;
                public static final int angleCurrentThreshold = 60;
                public static final double angleCurrentThresholdTime = 0.1;
                public static final boolean angleEnableCurrentLimit = true;

                public static final int driveCurrentLimit = 120;
                public static final int driveCurrentThreshold = 120;
                public static final double driveCurrentThresholdTime = 0.1;
                public static final boolean driveEnableCurrentLimit = true;

                // Ramp rates (unchanged)
                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                // Angle PID (steer Slot0Configs)
                public static final double angleKP = 100.0;
                public static final double angleKI = 0.0;
                public static final double angleKD = 0.5;

                // Drive PID (drive Slot0Configs)
                public static final double driveKP = 0.1;
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKF = 0.0;

                // Drive Feedforward
                public static final double driveKS = 0.0;
                public static final double driveKV = 0.124;
                public static final double driveKA = 0.0;

                // Max speed (kSpeedAt12Volts)
                public static final double maxSpeed = 5.85;

                public static final double driveRadius = Math.hypot(wheelBase, trackWidth) / 2.0;
                public static final double maxAngularVelocity = maxSpeed / driveRadius;

                // Neutral Modes
                public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
                public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

                /* ================= MODULE CONSTANTS ================= */

                /* Front Right Module - Module 0 */
                public static final class Mod0 {
                        public static final int driveMotorID = 3;
                        public static final int angleMotorID = 7;
                        public static final int canCoderID = 53;
                        public static final Rotation2d angleOffset =
                                Rotation2d.fromDegrees(-0.00732421875 * 360.0);
                        public static final SwerveModuleConstants constants =
                                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }

                /* Front Left Module - Module 1 */
                public static final class Mod1 {
                        public static final int driveMotorID = 0;
                        public static final int angleMotorID = 4;
                        public static final int canCoderID = 50;
                        public static final Rotation2d angleOffset =
                                Rotation2d.fromDegrees(-0.446044921875 * 360.0);
                        public static final SwerveModuleConstants constants =
                                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 {
                        public static final int driveMotorID = 1;
                        public static final int angleMotorID = 5;
                        public static final int canCoderID = 51;
                        public static final Rotation2d angleOffset =
                                Rotation2d.fromDegrees(-0.24365234375 * 360.0);
                        public static final SwerveModuleConstants constants =
                                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 {
                        public static final int driveMotorID = 2;
                        public static final int angleMotorID = 6;
                        public static final int canCoderID = 52;
                        public static final Rotation2d angleOffset =
                                Rotation2d.fromDegrees(0.15673828125 * 360.0);
                        public static final SwerveModuleConstants constants =
                                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
                }

                // Pathplanner offset (unchanged)
                public static final Translation2d front_offset =
                        new Translation2d(wheelBase / 2.0, trackWidth / 2.0);

                /* ================= AUTO CONSTANTS (UNCHANGED STRUCTURE) ================= */

                public static final class AutoConstants {
                        public static final double kMaxSpeedMetersPerSecond = 3.0;
                        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
                        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
                        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

                        public static final double kPXController = 1.0;
                        public static final double kPYController = 1.0;
                        public static final double kPThetaController = 1.0;

                        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                                new TrapezoidProfile.Constraints(
                                        kMaxAngularSpeedRadiansPerSecond,
                                        kMaxAngularSpeedRadiansPerSecondSquared);
                }

        }
        
        public static final class Shooter {
                // TODO: REMOVE DUMMY CONSTANTS
                public static final int shooterMotorID = 30;
                public static final InvertedValue shooterMotorInvert = InvertedValue.CounterClockwise_Positive;
                public static final NeutralModeValue shooterMotorIdleMode = NeutralModeValue.Coast;

                public static final int shooter2MotorID = 31;
                public static final InvertedValue shooter2MotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue shooter2MotorIdleMode = NeutralModeValue.Coast;
                public static final PIDController shooterPID = new PIDController(
                        0.1,
                        0,
                        0
                );
                
                public static final int hoodMotorID = 32;
                public static final InvertedValue hoodMotorInvert = InvertedValue.Clockwise_Positive;
                public static final NeutralModeValue hoodMotorIdleMode = NeutralModeValue.Brake;

                public static final int feederMotorID = 21;
                public static final boolean feederMotorInvert = false;
                public static final IdleMode feederMotorIdleMode = IdleMode.kBrake;

                public static final int spindexerMotorID = 20;
                public static final boolean spindexerMotorInvert = false;
                public static final IdleMode spindexerMotorIdleMode = IdleMode.kBrake;
                public static final int spindexerVelocity = 1;
                public static final PIDController spindexerPID = new PIDController(
                        0.1,
                        0,
                        0
                );

                public static final int maxVelocity = 3000;
                public static final double hubWidth = 47.0;
       }

        public static final class Vision {
                public static final String limelightName = "april";
        }

        public static final class Intake {
                // TODO: REMOVE DUMMY CONSTANTS
                public static final int intakeMotorID = 10;
                public static final boolean intakeMotorInvert = false;
                public static final IdleMode intakeMotorIdleMode = IdleMode.kBrake;
                public static final int elbowMotorID = 11;
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
        }
}