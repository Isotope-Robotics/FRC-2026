package frc.robot.lib.util;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConstants {
    public final int driveMotorId;
    public final int angleMotorId;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final InvertedValue driveMotorInvert;

    public SwerveModuleConstants(int driveMotorId, int angleMotorId, int cancoderID,
            Rotation2d angleOffset, InvertedValue driveMotorInvert) {
        this.driveMotorId = driveMotorId;
        this.angleMotorId = angleMotorId;
        this.cancoderID = cancoderID;
        this.angleOffset = angleOffset;
        this.driveMotorInvert = driveMotorInvert;
    }
}