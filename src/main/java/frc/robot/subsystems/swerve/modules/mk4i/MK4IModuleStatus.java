package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.ctre.PhoenixProUtils;

class MK4IModuleStatus {

    private final MK4IModuleSignals moduleSignals;

    protected MK4IModuleStatus(MK4IModuleSignals moduleSignals) {
        this.moduleSignals = moduleSignals;
    }

    protected StatusCode refreshEncoderSignals() {
        return BaseStatusSignal.refreshAll(
                getEncoderAbsolutePositionSignal(false),
                getEncoderVelocitySignal(false),
                getEncoderVoltageSignal(false)
        );
    }

    protected Rotation2d getEncoderAbsolutePosition(boolean refresh) {
        return Rotation2d.fromRotations(getEncoderAbsolutePositionSignal(refresh).getValue());
    }

    protected Rotation2d getEncoderVelocity(boolean refresh) {
        return Rotation2d.fromRotations(getEncoderVelocitySignal(refresh).getValue());
    }

    private StatusSignal<Double> getEncoderAbsolutePositionSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.encoderAbsolutePositionSignal());
    }

    protected StatusSignal<Double> getEncoderVelocitySignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.encoderVelocitySignal());
    }

    protected StatusSignal<Double> getEncoderVoltageSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.encoderVelocitySignal());
    }


    protected StatusCode refreshSteerMotorSignals() {
        return BaseStatusSignal.refreshAll(
                getSteerMotorPositionSignal(false),
                getSteerMotorVelocitySignal(false),
                getSteerMotorAccelerationSignal(false),
                getSteerMotorVoltageSignal(false)
        );
    }

    protected Rotation2d getSteerMotorLatencyPosition(boolean refresh) {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(
                getSteerMotorPositionSignal(refresh),
                getSteerMotorVelocitySignal(refresh)
        ));
    }

    protected Rotation2d getSteerMotorLatencyVelocity(boolean refresh) {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(
                getSteerMotorVelocitySignal(refresh),
                getSteerMotorAccelerationSignal(refresh)
        ));
    }

    protected Rotation2d getSteerMotorAcceleration(boolean refresh) {
        return Rotation2d.fromRotations(getSteerMotorAccelerationSignal(refresh).getValue());
    }

    protected StatusSignal<Double> getSteerMotorPositionSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerMotorPositionSignal());
    }

    protected StatusSignal<Double> getSteerMotorVelocitySignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerMotorVelocitySignal());
    }

    protected StatusSignal<Double> getSteerMotorAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerMotorAccelerationSignal());
    }

    protected StatusSignal<Double> getSteerMotorVoltageSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.steerMotorVoltageSignal());
    }


    protected StatusCode refreshDriveMotorSignals() {
        return BaseStatusSignal.refreshAll(
                getDriveMotorPositionSignal(false),
                getDriveMotorVelocitySignal(false),
                getDriveMotorAccelerationSignal(false),
                getDriveMotorVoltageSignal(false),
                getDriveMotorStatorCurrentSignal(false)
        );
    }

    protected Rotation2d getDriveMotorLatencyPosition(boolean refresh) {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(
                getDriveMotorPositionSignal(refresh),
                getDriveMotorVelocitySignal(refresh)
        ));
    }

    protected Rotation2d getDriveMotorLatencyVelocity(boolean refresh) {
        return Rotation2d.fromRotations(BaseStatusSignal.getLatencyCompensatedValue(
                getDriveMotorVelocitySignal(refresh),
                getDriveMotorAccelerationSignal(refresh)
        ));
    }

    protected Rotation2d getDriveMotorAcceleration(boolean refresh) {
        return Rotation2d.fromRotations(getDriveMotorAccelerationSignal(refresh).getValue());
    }

    protected StatusSignal<Double> getDriveMotorPositionSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.drivePositionSignal());
    }

    protected StatusSignal<Double> getDriveMotorVelocitySignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.driveVelocitySignal());
    }

    protected StatusSignal<Double> getDriveMotorAccelerationSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.driveAccelerationSignal());
    }

    protected StatusSignal<Double> getDriveMotorVoltageSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.driveVoltageSignal());
    }

    protected StatusSignal<Double> getDriveMotorStatorCurrentSignal(boolean refresh) {
        return PhoenixProUtils.getRefreshedSignal(refresh, moduleSignals.driveStatorCurrentSignal());
    }

}
