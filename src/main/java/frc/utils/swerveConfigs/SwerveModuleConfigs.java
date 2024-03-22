package frc.utils.swerveConfigs;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModuleConfigs {

    public final int driveMotorID;
    public final int angleMotorID;
    public final int cancoderID;
    public final Rotation2d angleOffset;
    public final CTREConfigs ctreConfigs;

    public SwerveModuleConfigs(int driveMotorID, int angleMotorID, int canCoderID) {
        this(driveMotorID,angleMotorID,canCoderID,Rotation2d.fromDegrees(0));
    }

    public SwerveModuleConfigs(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset) {
        this(driveMotorID,angleMotorID,canCoderID,angleOffset,new CTREConfigs());
    }

    public SwerveModuleConfigs(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, boolean isDriveInverted, boolean isAngleInverted) {
        this(driveMotorID,angleMotorID,canCoderID,angleOffset,
                new CTREConfigs(
                    isDriveInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive,
                    isAngleInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive,
                    isAngleInverted ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive
            )
        );
    }

    public SwerveModuleConfigs(int driveMotorID, int angleMotorID, int canCoderID, Rotation2d angleOffset, CTREConfigs ctreConfigs) {
        this.driveMotorID = driveMotorID;
        this.angleMotorID = angleMotorID;
        this.cancoderID = canCoderID;
        this.angleOffset = angleOffset;
        this.ctreConfigs = ctreConfigs;
    }
}