package frc.robot.subsystems.swerve.modules.mk4imodule;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.utils.devicewrappers.TalonFXWrapper;

class MK4IModuleActions {

    private final TalonFXWrapper steerMotor, driveMotor;


    private final VoltageOut driveVoltageRequest =
            new VoltageOut(0).withEnableFOC(ModuleConstants.ENABLE_FOC_DRIVE);

    private final VelocityVoltage driveVelocityRequest =
            new VelocityVoltage(0).withEnableFOC(ModuleConstants.ENABLE_FOC_DRIVE);

    private final PositionVoltage steerPositionRequest =
            new PositionVoltage(0).withEnableFOC(ModuleConstants.ENABLE_FOC_STEER);

    private final VoltageOut steerVoltageRequest =
            new VoltageOut(0).withEnableFOC(ModuleConstants.ENABLE_FOC_STEER);

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


    public void setTargetDriveVoltage(double voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }

    public void setTargetSteerVoltage(double voltage) {
        steerMotor.setControl(steerVoltageRequest.withOutput(voltage));
    }

    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

}
