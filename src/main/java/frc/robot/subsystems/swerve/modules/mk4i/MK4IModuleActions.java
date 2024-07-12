package frc.robot.subsystems.swerve.modules.mk4i;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.devicewrappers.TalonFXWrapper;

class MK4IModuleActions {

    private final TalonFXWrapper steerMotor, driveMotor;

    //todo - VelocityTorqueCurrentFOC (whats better)
    private final VelocityVoltage driveVelocityRequest;
    private final VoltageOut driveVoltageRequest;

    //todo - MotionMagicExpoTorqueCurrentFOC (whats better)
    private final PositionVoltage steerPositionRequest;
    private final VoltageOut steerVoltageRequest;

    public MK4IModuleActions(MK4IModuleConstants constants) {
        this.driveMotor = constants.driveMotor();
        this.steerMotor = constants.steerMotor();

        this.driveVelocityRequest =  new VelocityVoltage(0).withEnableFOC(constants.enableFocDrive());
        this.driveVoltageRequest = new VoltageOut(0).withEnableFOC(constants.enableFocDrive());

        this.steerPositionRequest = new PositionVoltage(0).withEnableFOC(constants.enableFocSteer());
        this.steerVoltageRequest = new VoltageOut(0).withEnableFOC(constants.enableFocSteer());
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


    public void setTargetDriveVoltage(double voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    public void setTargetSteerVoltage(double voltage) {
        steerMotor.setControl(steerVoltageRequest.withOutput(voltage));
    }


    public void setTargetClosedLoopVelocity(double targetVelocityRotationsPerSecond) {
        driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

}
