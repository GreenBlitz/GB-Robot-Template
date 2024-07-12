package frc.robot.subsystems.swerve.modules.mk4i;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.Conversions;

import java.util.Queue;

public class MK4IModule implements IModule {

    private final MK4IModuleStatus status;
    private final MK4IModuleActions actions;
    private final MK4IModuleConstants constants;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    public MK4IModule(MK4IModuleConstants constants) {
        this.constants = constants;
        this.status = new MK4IModuleStatus(constants.signals());
        this.actions = new MK4IModuleActions(constants);

        this.steerPositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                constants.steerMotor(),
                status.getSteerMotorPositionSignal(false),
                status.getSteerMotorVelocitySignal(false)
        );
        this.drivePositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                constants.driveMotor(),
                status.getDriveMotorPositionSignal(false),
                status.getDriveMotorVelocitySignal(false)
        );

        actions.resetDriveAngle(new Rotation2d());
    }

    private double toDriveMeters(double rotations) {
        return toDriveMeters(Rotation2d.fromRotations(rotations));
    }

    private double toDriveMeters(Rotation2d angle) {
        return Conversions.angleToDistance(angle, constants.wheelDiameterMeters());
    }


    @Override
    public void stop() {
        actions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        actions.setBrake(brake);
    }

    @Override
    public void resetByEncoder() {
        actions.resetSteerAngle(status.getEncoderAbsolutePosition(true));
    }


    @Override
    public void runSteerMotorByVoltage(double voltage) {
        actions.setTargetSteerVoltage(voltage);
    }

    @Override
    public void runDriveMotorByVoltage(double voltage) {
        actions.setTargetDriveVoltage(voltage);
    }


    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                status.getSteerMotorLatencyVelocity(true),
                constants.couplingRatio(),
                constants.maxVelocityPerSecond(),
                constants.wheelDiameterMeters(),
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        actions.setTargetDriveVoltage(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(
                targetVelocityMetersPerSecond,
                constants.wheelDiameterMeters()
        );
        double optimizedVelocityRevolutionsPerSecond = ModuleUtils.removeCouplingFromRevolutions(
                targetVelocityPerSecond,
                status.getSteerMotorLatencyVelocity(true),
                constants.couplingRatio()
        );
        actions.setTargetClosedLoopVelocity(optimizedVelocityRevolutionsPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        actions.setTargetAngle(angle);
    }


    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        inputs.getEncoderInputs().isConnected = status.refreshEncoderSignals().isOK();
        inputs.getEncoderInputs().angle = status.getEncoderAbsolutePosition(false);
        inputs.getEncoderInputs().velocity = status.getEncoderVelocity(false);
        inputs.getEncoderInputs().voltage = status.getEncoderVoltageSignal(false).getValue();

        inputs.getSteerMotorInputs().isConnected = status.refreshSteerMotorSignals().isOK();
        inputs.getSteerMotorInputs().angle = status.getSteerMotorLatencyPosition(false);
        inputs.getSteerMotorInputs().velocity = status.getSteerMotorLatencyVelocity(false);
        inputs.getSteerMotorInputs().acceleration = status.getSteerMotorAcceleration(false);
        inputs.getSteerMotorInputs().voltage = status.getSteerMotorVoltageSignal(false).getValue();
        inputs.getSteerMotorInputs().angleOdometrySamples = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();

        inputs.getDriveMotorInputs().isConnected = status.refreshDriveMotorSignals().isOK();
        inputs.getDriveMotorInputs().angle = status.getDriveMotorLatencyPosition(false);
        inputs.getDriveMotorInputs().velocity = status.getDriveMotorLatencyVelocity(false);
        inputs.getDriveMotorInputs().acceleration = status.getDriveMotorAcceleration(false);
        inputs.getDriveMotorInputs().current = status.getDriveMotorStatorCurrentSignal(false).getValue();
        inputs.getDriveMotorInputs().voltage = status.getDriveMotorVoltageSignal(false).getValue();
        inputs.getDriveMotorInputs().distanceMeters = toDriveMeters(inputs.getDriveMotorInputs().angle);
        inputs.getDriveMotorInputs().velocityMeters = toDriveMeters(inputs.getDriveMotorInputs().velocity);
        inputs.getDriveMotorInputs().distanceMetersOdometrySamples = drivePositionQueue.stream().mapToDouble(this::toDriveMeters).toArray();
        drivePositionQueue.clear();
    }

}
