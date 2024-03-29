package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleConstants;
import frc.utils.Conversions;

public class MK4IModuleData {

    private final StatusSignal<Double>
            steerEncoderAbsolutePositionSignal, steerEncoderVelocitySignal, steerEncoderVoltageSignal;
    private final StatusSignal<Double>
            driveStatorCurrentSignal, drivePositionSignal, driveVelocitySignal, driveAccelerationSignal, driveVoltageSignal;
    private final StatusSignal<Double>
            steerPositionSignal, steerVelocitySignal, steerAccelerationSignal, steerVoltageSignal;


    private static double toDriveDistance(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, ModuleConstants.WHEEL_DIAMETER_METERS);
    }

    public MK4IModuleData(MK4IModuleConfigObject mk4IModuleConfigObject){
        steerEncoderVoltageSignal = mk4IModuleConfigObject.steerEncoderVoltageSignal;
        steerEncoderVelocitySignal = mk4IModuleConfigObject.steerEncoderVelocitySignal;
        steerEncoderAbsolutePositionSignal = mk4IModuleConfigObject.steerEncoderAbsolutePositionSignal;

        driveVoltageSignal = mk4IModuleConfigObject.driveVoltageSignal;
        driveStatorCurrentSignal = mk4IModuleConfigObject.driveStatorCurrentSignal;
        driveAccelerationSignal = mk4IModuleConfigObject.driveAccelerationSignal;
        driveVelocitySignal = mk4IModuleConfigObject.driveVelocitySignal;
        drivePositionSignal = mk4IModuleConfigObject.drivePositionSignal;

        steerVoltageSignal = mk4IModuleConfigObject.steerVoltageSignal;
        steerAccelerationSignal = mk4IModuleConfigObject.steerAccelerationSignal;
        steerVelocitySignal = mk4IModuleConfigObject.steerVelocitySignal;
        steerPositionSignal = mk4IModuleConfigObject.steerPositionSignal;
    }


    public Rotation2d getEncoderAbsolutePosition(){
        return Rotation2d.fromRotations(getSteerEncoderAbsolutePositionSignal().getValue());
    }
    public double getDriveMotorVelocityInMeters(){
        return toDriveDistance(getDriveMotorLatencyVelocity());
    }
    public double getDriveMotorPositionInMeters(){
        return toDriveDistance(getDriveMotorLatencyPosition());
    }
    public Rotation2d getSteerMotorVelocity(){
        return Rotation2d.fromRotations(getSteerMotorLatencyVelocity());
    }
    public Rotation2d getSteerMotorPosition(){
        return Rotation2d.fromRotations(getSteerMotorLatencyPosition());
    }


    private double getSteerMotorLatencyVelocity(){
        return BaseStatusSignal.getLatencyCompensatedValue(getSteerVelocitySignal(), getSteerAccelerationSignal());
    }
    private double getSteerMotorLatencyPosition(){
        return BaseStatusSignal.getLatencyCompensatedValue(getSteerPositionSignal(), getSteerVelocitySignal());
    }
    private double getDriveMotorLatencyVelocity(){
        return BaseStatusSignal.getLatencyCompensatedValue(getDriveVelocitySignal(), getDriveAccelerationSignal());
    }
    private double getDriveMotorLatencyPosition(){
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


