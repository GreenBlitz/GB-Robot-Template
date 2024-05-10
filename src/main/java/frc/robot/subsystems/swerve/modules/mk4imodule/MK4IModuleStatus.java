package frc.robot.subsystems.swerve.modules.mk4imodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.PhoenixProUtils;

class MK4IModuleStatus {

    private final MK4IModuleRecords.MK4IModuleSignals moduleSignals;

    public MK4IModuleStatus(MK4IModuleRecords.MK4IModuleSignals moduleSignals) {
        this.moduleSignals = moduleSignals;
    }


    // Encoder Status
    public Rotation2d getSteerEncoderAbsolutePosition(boolean refresh) {
        return Rotation2d.fromRotations(getSteerEncoderAbsolutePositionSignal(refresh).getValue());
    }

    private StatusSignal<Double> getSteerEncoderAbsolutePositionSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerEncoderAbsolutePositionSignal());
    }

    public StatusSignal<Double> getSteerEncoderVelocitySignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerEncoderVelocitySignal());
    }

    public StatusSignal<Double> getSteerEncoderVoltageSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerEncoderVelocitySignal());
    }


    // Drive Motor Status
    public Rotation2d getDriveMotorLatencyPosition(boolean refresh) {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(
                getDriveMotorPositionSignal(refresh),
                getDriveMotorVelocitySignal(refresh)
        ));
    }

    public Rotation2d getDriveMotorLatencyVelocity(boolean refresh) {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(
                getDriveMotorVelocitySignal(refresh),
                getDriveMotorAccelerationSignal(refresh)
        ));
    }

    public Rotation2d getDriveMotorAcceleration(boolean refresh) {
        return Rotation2d.fromRotations(getDriveMotorAccelerationSignal(refresh).getValue());
    }

    public StatusSignal<Double> getDriveMotorPositionSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.drivePositionSignal());
    }

    public StatusSignal<Double> getDriveMotorVelocitySignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.driveVelocitySignal());
    }

    public StatusSignal<Double> getDriveMotorAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.driveAccelerationSignal());
    }

    public StatusSignal<Double> getDriveMotorVoltageSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.driveVoltageSignal());
    }

    public StatusSignal<Double> getDriveMotorStatorCurrentSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.driveStatorCurrentSignal());
    }


    // Steer Motor Status
    public Rotation2d getSteerMotorLatencyPosition(boolean refresh) {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(
                getSteerMotorPositionSignal(refresh),
                getSteerMotorVelocitySignal(refresh)
        ));
    }

    public Rotation2d getSteerMotorLatencyVelocity(boolean refresh) {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(
                getSteerMotorVelocitySignal(refresh),
                getSteerMotorAccelerationSignal(refresh)
        ));
    }

    public Rotation2d getSteerMotorAcceleration(boolean refresh) {
        return Rotation2d.fromRotations(getSteerMotorAccelerationSignal(refresh).getValue());
    }

    public StatusSignal<Double> getSteerMotorPositionSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerMotorPositionSignal());
    }

    public StatusSignal<Double> getSteerMotorVelocitySignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerMotorVelocitySignal());
    }

    public StatusSignal<Double> getSteerMotorAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerMotorAccelerationSignal());
    }

    public StatusSignal<Double> getSteerMotorVoltageSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerMotorVoltageSignal());
    }


    // Refresh All
    public StatusCode refreshAllSignals() {
        return BaseStatusSignal.refreshAll(
                getSteerEncoderAbsolutePositionSignal(false),
                getSteerEncoderVelocitySignal(false),
                getSteerEncoderVoltageSignal(false),

                getDriveMotorPositionSignal(false),
                getDriveMotorVelocitySignal(false),
                getDriveMotorAccelerationSignal(false),
                getDriveMotorVoltageSignal(false),
                getDriveMotorStatorCurrentSignal(false),

                getSteerMotorPositionSignal(false),
                getSteerMotorVelocitySignal(false),
                getSteerMotorAccelerationSignal(false),
                getSteerMotorVoltageSignal(false)
        );
    }

}
