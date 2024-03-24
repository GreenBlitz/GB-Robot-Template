package frc.robot.subsystems.swerve.mk4iswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;
import frc.utils.Conversions;
import frc.utils.devicewrappers.GBTalonFXPro;

import java.util.Queue;

public class MK4IModule implements IModule {

    private final GBTalonFXPro steerMotor, driveMotor;
    private final CANcoder steerEncoder;
    private final MK4IModuleConfigObject moduleConfigObject;
    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(MK4IModuleConstants.ENABLE_FOC);

    public MK4IModule(MK4IModuleConfigObject moduleConfigObject) {
        this.steerMotor = moduleConfigObject.getSteerMotor();
        this.driveMotor = moduleConfigObject.getDriveMotor();
        this.steerEncoder = moduleConfigObject.getSteerEncoder();
        this.moduleConfigObject = moduleConfigObject;
        this.steerPositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(steerMotor, moduleConfigObject.steerPositionSignal);
        this.drivePositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(driveMotor, moduleConfigObject.drivePositionSignal);
    }

    private double getAngleDegrees() {
        return Conversions.revolutionsToDegrees(moduleConfigObject.steerPositionSignal.getValue());
    }

    private double toDriveDistance(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, MK4IModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    public void setTargetOpenLoopVelocity(double voltage) {
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        driveMotor.setControl(driveVelocityRequest.withVelocity(targetVelocityMetersPerSecond));
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    @Override
    public void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    public void setBrake(boolean brake) {
        final NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
        steerMotor.setNeutralMode(neutralModeValue);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                moduleConfigObject.steerPositionSignal,
                moduleConfigObject.steerVelocitySignal,
                moduleConfigObject.drivePositionSignal,
                moduleConfigObject.driveVelocitySignal,
                moduleConfigObject.driveStatorCurrentSignal,
                moduleConfigObject.steerEncoderPositionSignal,
                moduleConfigObject.steerEncoderVelocitySignal
        );
    }

    @Override
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.steerEncoderAngleDegrees = Conversions.revolutionsToDegrees(steerEncoder.getAbsolutePosition().getValue());
        inputs.steerEncoderVelocity = steerEncoder.getVelocity().getValue();
        inputs.steerEncoderVoltage = steerEncoder.getSupplyVoltage().getValue();

        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray();
        inputs.steerVoltage = moduleConfigObject.steerVoltageSignal.getValue();
        inputs.steerVelocity = steerMotor.getLatencyCompensatedVelocity();

        inputs.driveDistanceMeters = toDriveDistance(driveMotor.getLatencyCompensatedPosition());
        inputs.odometryUpdatesDriveDistanceMeters = drivePositionQueue.stream().mapToDouble(this::toDriveDistance).toArray();
        inputs.driveVelocityMetersPerSecond = toDriveDistance(driveMotor.getLatencyCompensatedVelocity());
        inputs.driveCurrent = moduleConfigObject.driveStatorCurrentSignal.getValue();
        inputs.driveVoltage = moduleConfigObject.driveVoltageSignal.getValue();

        steerPositionQueue.clear();
        drivePositionQueue.clear();
    }

}
