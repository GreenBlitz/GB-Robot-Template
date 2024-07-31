package frc.robot.subsystems.swerve.modules.talonfx;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.devicewrappers.TalonFXWrapper;
import frc.robot.subsystems.swerve.modules.talonfx.TalonFXModuleConstants.CloseLoopOutputType;

class TalonFXModuleActions {

    private final TalonFXWrapper steerMotor, driveMotor;

    //todo - MotionMagicExpoTorqueCurrentFOC (whats better)
    private final PositionVoltage steerPositionRequest;
    private final VoltageOut steerVoltageRequest;

    private final VelocityVoltage driveVelocityVoltageRequest;
    private final VoltageOut driveVoltageRequest;
    private final VelocityTorqueCurrentFOC driveVelocityTorqueRequest;
    private final TorqueCurrentFOC driveTorqueRequest;

    protected TalonFXModuleActions(TalonFXModuleConstants constants) {
        this.steerMotor = constants.getSteerMotor();
        this.driveMotor = constants.getDriveMotor();

        this.steerPositionRequest = new PositionVoltage(0).withEnableFOC(constants.getEnableFOCSteer());
        this.steerVoltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOCSteer());

        this.driveVelocityVoltageRequest =  new VelocityVoltage(0).withEnableFOC(constants.getEnableFOCDrive());
        this.driveVoltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOCDrive());
        this.driveVelocityTorqueRequest = new VelocityTorqueCurrentFOC(0).withSlot(1);
        this.driveTorqueRequest = new TorqueCurrentFOC(0);
    }

    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }
    protected void setBrake(boolean brake) {
        NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        steerMotor.setNeutralMode(neutralModeValue);
        driveMotor.setNeutralMode(neutralModeValue);
    }


    protected void resetSteerAngle(Rotation2d angle) {
        steerMotor.setPosition(angle.getRotations());
    }
    protected void resetDriveAngle(Rotation2d angle) {
        driveMotor.setPosition(angle.getRotations());
    }


    protected void setTargetSteerVoltage(double voltage) {
        steerMotor.setControl(steerVoltageRequest.withOutput(voltage));
    }
    protected void setTargetDriveOutputValue(double value, CloseLoopOutputType closeLoopOutputType) {
        ControlRequest controlRequest = switch (closeLoopOutputType) {
            case VOLTAGE -> driveVoltageRequest.withOutput(value);
            case TORQUE_CURRENT -> driveTorqueRequest.withOutput(value);
        };
        driveMotor.setControl(controlRequest);
    }


    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }
    protected void setTargetClosedLoopVelocity(Rotation2d targetVelocityPerSecond, CloseLoopOutputType closeLoopOutputType) {
        ControlRequest controlRequest = switch (closeLoopOutputType) {
            case VOLTAGE -> driveVelocityVoltageRequest.withVelocity(targetVelocityPerSecond.getRotations());
            case TORQUE_CURRENT -> driveVelocityTorqueRequest.withVelocity(targetVelocityPerSecond.getRotations());
        };
        driveMotor.setControl(controlRequest);
    }

}
