package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleUtils;

class MK4IModuleStatus {

    private final MK4IModuleRecords.MK4IModuleSignals moduleSignals;

    public MK4IModuleStatus(MK4IModuleRecords.MK4IModuleSignals moduleSignals) {
        this.moduleSignals = moduleSignals;
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
        return moduleSignals.steerEncoderVoltageSignal().refresh();
    }

    public StatusSignal<Double> getSteerEncoderVelocitySignal() {
        return moduleSignals.steerEncoderVelocitySignal().refresh();
    }

    private StatusSignal<Double> getSteerEncoderAbsolutePositionSignal() {
        return moduleSignals.steerEncoderAbsolutePositionSignal().refresh();
    }

    public StatusSignal<Double> getDriveVoltageSignal() {
        return moduleSignals.driveVoltageSignal().refresh();
    }

    public StatusSignal<Double> getDriveStatorCurrentSignal() {
        return moduleSignals.driveStatorCurrentSignal().refresh();
    }

    private StatusSignal<Double> getDriveAccelerationSignal() {
        return moduleSignals.driveAccelerationSignal().refresh();
    }

    private StatusSignal<Double> getDriveVelocitySignal() {
        return moduleSignals.driveVelocitySignal().refresh();
    }

    private StatusSignal<Double> getDrivePositionSignal() {
        return moduleSignals.drivePositionSignal().refresh();
    }

    public StatusSignal<Double> getSteerVoltageSignal() {
        return moduleSignals.steerVoltageSignal().refresh();
    }

    private StatusSignal<Double> getSteerAccelerationSignal() {
        return moduleSignals.steerAccelerationSignal().refresh();
    }

    private StatusSignal<Double> getSteerVelocitySignal() {
        return moduleSignals.steerVelocitySignal().refresh();
    }

    private StatusSignal<Double> getSteerPositionSignal() {
        return moduleSignals.steerPositionSignal().refresh();
    }
}
