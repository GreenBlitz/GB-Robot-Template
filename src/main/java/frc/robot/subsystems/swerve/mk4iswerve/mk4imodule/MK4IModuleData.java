package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleUtils;

public class MK4IModuleData {

    private final StatusSignal<Double> steerEncoderAbsolutePositionSignal,
            steerEncoderVelocitySignal,
            steerEncoderVoltageSignal;
    private final StatusSignal<Double> driveStatorCurrentSignal,
            drivePositionSignal,
            driveVelocitySignal,
            driveAccelerationSignal,
            driveVoltageSignal;
    private final StatusSignal<Double> steerPositionSignal,
            steerVelocitySignal,
            steerAccelerationSignal,
            steerVoltageSignal;

    public MK4IModuleData(MK4IModuleConfigObject mk4IModuleConfigObject) {
        this.steerEncoderVoltageSignal = mk4IModuleConfigObject.steerEncoderVoltageSignal;
        this.steerEncoderVelocitySignal = mk4IModuleConfigObject.steerEncoderVelocitySignal;
        this.steerEncoderAbsolutePositionSignal = mk4IModuleConfigObject.steerEncoderAbsolutePositionSignal;

        this.driveVoltageSignal = mk4IModuleConfigObject.driveVoltageSignal;
        this.driveStatorCurrentSignal = mk4IModuleConfigObject.driveStatorCurrentSignal;
        this.driveAccelerationSignal = mk4IModuleConfigObject.driveAccelerationSignal;
        this.driveVelocitySignal = mk4IModuleConfigObject.driveVelocitySignal;
        this.drivePositionSignal = mk4IModuleConfigObject.drivePositionSignal;

        this.steerVoltageSignal = mk4IModuleConfigObject.steerVoltageSignal;
        this.steerAccelerationSignal = mk4IModuleConfigObject.steerAccelerationSignal;
        this.steerVelocitySignal = mk4IModuleConfigObject.steerVelocitySignal;
        this.steerPositionSignal = mk4IModuleConfigObject.steerPositionSignal;
    }

    public Rotation2d getEncoderAbsolutePosition() {
        return Rotation2d.fromRotations(getSteerEncoderAbsolutePositionSignal().getValue());
    }

    public double getDriveMotorVelocityInMeters() {
        return ModuleUtils.toDriveDistance(getDriveMotorLatencyVelocity());
    }

    public double getDriveMotorPositionInMeters() {
        return ModuleUtils.toDriveDistance(getDriveMotorLatencyPosition());
    }

    public Rotation2d getSteerMotorVelocity() {
        return Rotation2d.fromRotations(getSteerMotorLatencyVelocity());
    }

    public Rotation2d getSteerMotorPosition() {
        return Rotation2d.fromRotations(getSteerMotorLatencyPosition());
    }

    private double getSteerMotorLatencyVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(getSteerVelocitySignal(), getSteerAccelerationSignal());
    }

    private double getSteerMotorLatencyPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(getSteerPositionSignal(), getSteerVelocitySignal());
    }

    private double getDriveMotorLatencyVelocity() {
        return BaseStatusSignal.getLatencyCompensatedValue(getDriveVelocitySignal(), getDriveAccelerationSignal());
    }

    private double getDriveMotorLatencyPosition() {
        return BaseStatusSignal.getLatencyCompensatedValue(getDrivePositionSignal(), getDriveVelocitySignal());
    }

    public StatusSignal<Double> getSteerEncoderVoltageSignal() {
        return steerEncoderVoltageSignal.refresh();
    }

    public StatusSignal<Double> getSteerEncoderVelocitySignal() {
        return steerEncoderVelocitySignal.refresh();
    }

    private StatusSignal<Double> getSteerEncoderAbsolutePositionSignal() {
        return steerEncoderAbsolutePositionSignal.refresh();
    }

    public StatusSignal<Double> getDriveVoltageSignal() {
        return driveVoltageSignal.refresh();
    }

    public StatusSignal<Double> getDriveStatorCurrentSignal() {
        return driveStatorCurrentSignal.refresh();
    }

    private StatusSignal<Double> getDriveAccelerationSignal() {
        return driveAccelerationSignal.refresh();
    }

    private StatusSignal<Double> getDriveVelocitySignal() {
        return driveVelocitySignal.refresh();
    }

    private StatusSignal<Double> getDrivePositionSignal() {
        return drivePositionSignal.refresh();
    }

    public StatusSignal<Double> getSteerVoltageSignal() {
        return steerVoltageSignal.refresh();
    }

    private StatusSignal<Double> getSteerAccelerationSignal() {
        return steerAccelerationSignal.refresh();
    }

    private StatusSignal<Double> getSteerVelocitySignal() {
        return steerVelocitySignal.refresh();
    }

    private StatusSignal<Double> getSteerPositionSignal() {
        return steerPositionSignal.refresh();
    }
}
