package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.utils.devicewrappers.GBTalonFXPro;

public class MK4IActions {

    private final GBTalonFXPro steerMotor, driveMotor;

    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC_DRIVE);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC_STEER);


    public MK4IActions(GBTalonFXPro driveMotor, GBTalonFXPro steerMotor){
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }


    public void setTargetOpenLoopVelocity(double voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }


    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }


    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }


    public void resetSteerAngle(Rotation2d angle) {
        steerMotor.setPosition(angle.getRotations());
    }


    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }


    public void setBrake(boolean brake) {
        final NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
        steerMotor.setNeutralMode(neutralModeValue);
    }


}
