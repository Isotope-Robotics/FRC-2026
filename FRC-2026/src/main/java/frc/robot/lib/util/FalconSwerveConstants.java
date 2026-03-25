package frc.robot.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;

public class FalconSwerveConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final InvertedValue driveMotorInvert;
    public final InvertedValue angleMotorInvert;
    public final SensorDirectionValue cancoderInvert;

    public FalconSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio,
            double angleKP, double angleKI, double angleKD,
            InvertedValue driveMotorInvert, InvertedValue angleMotorInvert,
            SensorDirectionValue cancoderInvert) {
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = cancoderInvert;
    }

    public static final class SDS {
        public static final class MK5n {
            // MK5n steering gear ratio: 287:11 = 26.0909...
            // MK5n bevel gears face OUTWARD (opposite of MK4i)
            // This is why offsets need +90 correction vs MK4i procedure

            public static FalconSwerveConstants Falcon500(double driveGearRatio) {
                double wheelDiameter = Units.inchesToMeters(4.0);
                double angleGearRatio = (287.0 / 11.0); // 26.09:1

                double angleKP = 100.0;
                double angleKI = 0.0;
                double angleKD = 0.5;

                // CounterClockwise_Positive on drive, Clockwise_Positive on steer
                // is correct for MK5n with Falcon 500
                InvertedValue driveMotorInvert = InvertedValue.CounterClockwise_Positive;
                InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
                SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

                return new FalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio,
                        angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert);
            }

            public static FalconSwerveConstants Falcon500Inverted(double driveGearRatio) {
                double wheelDiameter = Units.inchesToMeters(4.0);
                double angleGearRatio = (287.0 / 11.0);

                double angleKP = 100.0;
                double angleKI = 0.0;
                double angleKD = 0.5;

                InvertedValue driveMotorInvert = InvertedValue.Clockwise_Positive;
                InvertedValue angleMotorInvert = InvertedValue.Clockwise_Positive;
                SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

                return new FalconSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio,
                        angleKP, angleKI, angleKD, driveMotorInvert, angleMotorInvert, cancoderInvert);
            }

            public static final class driveRatios {
                public static final double L1 = (7.03 / 1.0);
                public static final double L2 = (6.03 / 1.0);
                public static final double L3 = (5.27 / 1.0);
            }
        }

        // Keep MK4i alias pointing to MK5n so existing Constants.java references compile
        public static final class MK4i {
            public static FalconSwerveConstants Falcon500(double driveGearRatio) {
                return MK5n.Falcon500(driveGearRatio);
            }
            public static FalconSwerveConstants Falcon500Inverted(double driveGearRatio) {
                return MK5n.Falcon500Inverted(driveGearRatio);
            }
            public static final class driveRatios {
                public static final double L1 = MK5n.driveRatios.L1;
                public static final double L2 = MK5n.driveRatios.L2;
                public static final double L3 = MK5n.driveRatios.L3;
            }
        }
    }
}