package frc.robot.subsystems.swerve.modules.talonfx;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.devicewrappers.TalonFXWrapper;

class TalonFXModuleActions {

    private final TalonFXWrapper steerMotor, driveMotor;

    //todo - MotionMagicExpoTorqueCurrentFOC (whats better)
    private final PositionVoltage steerPositionRequest;
    private final VoltageOut steerVoltageRequest;

    //todo - VelocityTorqueCurrentFOC (whats better)
    private final VelocityVoltage driveVelocityRequest;
    private final VoltageOut driveVoltageRequest;

    protected TalonFXModuleActions(TalonFXModuleConstants constants) {
        this.steerMotor = constants.getSteerMotor();
        this.driveMotor = constants.getDriveMotor();

        this.steerPositionRequest = new PositionVoltage(0).withEnableFOC(constants.getEnableFOCSteer());
        this.steerVoltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOCSteer());

        this.driveVelocityRequest =  new VelocityVoltage(0).withEnableFOC(constants.getEnableFOCDrive());
        this.driveVoltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOCDrive());
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
    protected void setTargetDriveVoltage(double voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }


    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }
    protected void setTargetClosedLoopVelocity(double targetVelocityRotationsPerSecond) {
        driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

}
