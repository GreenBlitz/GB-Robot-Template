package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.devicewrappers.TalonFXWrapper;

class MK4IModuleActions {
    public enum ClosedLoopOutputType {
        TorqueCurrentFOC,
        Voltage
    }

    private final TalonFXWrapper steerMotor, driveMotor;

    private final VelocityVoltage driveVoltageVelocityRequest = new VelocityVoltage(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC_DRIVE);
    private final VelocityTorqueCurrentFOC driveCurrentVelocityRequest = new VelocityTorqueCurrentFOC(0).withSlot(1).withOverrideCoastDurNeutral(true);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC_DRIVE);
    private final TorqueCurrentFOC driveCurrentRequest = new TorqueCurrentFOC(0).withOverrideCoastDurNeutral(true);

    private final PositionVoltage steerVoltagePositionRequest = new PositionVoltage(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC_STEER);
    private final VoltageOut steerVoltageRequest = new VoltageOut(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC_STEER);

    public MK4IModuleActions(MK4IModuleRecords.MK4IModuleMotors moduleMotors) {
        this.driveMotor = moduleMotors.driveMotor();
        this.steerMotor = moduleMotors.steerMotor();
    }

    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    public void setBrake(boolean brake) {
        NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
        steerMotor.setNeutralMode(neutralModeValue);
    }


    public void resetDriveAngle(Rotation2d angle) {
        driveMotor.setPosition(angle.getRotations());
    }

    public void resetSteerAngle(Rotation2d angle) {
        steerMotor.setPosition(angle.getRotations());
    }


    public void setTargetDriveOutputValue(double value, ClosedLoopOutputType closedLoopOutputType) {
        ControlRequest controlRequest = switch (closedLoopOutputType) {
            case Voltage -> driveVoltageRequest.withOutput(value);
            case TorqueCurrentFOC -> driveCurrentRequest.withOutput(value);
        };
        driveMotor.setControl(controlRequest);
    }

    public void setTargetSteerVoltage(double voltage) {
        steerMotor.setControl(steerVoltageRequest.withOutput(voltage));
    }


    public void setTargetClosedLoopVelocity(double targetVelocityRotationsPerSecond, ClosedLoopOutputType closedLoopOutputType) {
        ControlRequest controlRequest = switch (closedLoopOutputType) {
            case Voltage -> driveVoltageVelocityRequest.withVelocity(targetVelocityRotationsPerSecond);
            case TorqueCurrentFOC -> driveCurrentVelocityRequest.withVelocity(targetVelocityRotationsPerSecond);
        };
        driveMotor.setControl(controlRequest);
    }

    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerVoltagePositionRequest.withPosition(angle.getRotations()));
    }

}
