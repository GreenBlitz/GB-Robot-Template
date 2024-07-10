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

    private final MK4IModuleStatus mk4iModuleStatus;
    private final MK4IModuleActions mk4iModuleActions;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    public MK4IModule(MK4IModuleConfigObject moduleConfigObject) {
        MK4IModuleRecords.MK4IModuleMotors motors = moduleConfigObject.getMotors();

        this.mk4iModuleStatus = new MK4IModuleStatus(moduleConfigObject.getModuleSignals());
        this.mk4iModuleActions = new MK4IModuleActions(motors);

        this.steerPositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                motors.steerMotor(),
                mk4iModuleStatus.getSteerMotorPositionSignal(false),
                mk4iModuleStatus.getSteerMotorVelocitySignal(false)
        );
        this.drivePositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                motors.driveMotor(),
                mk4iModuleStatus.getDriveMotorPositionSignal(false),
                mk4iModuleStatus.getDriveMotorVelocitySignal(false)
        );

        mk4iModuleActions.resetDriveAngle(new Rotation2d());
    }

    private double toDriveMeters(double rotations) {
        return toDriveMeters(Rotation2d.fromRotations(rotations));
    }

    private double toDriveMeters(Rotation2d angle) {
        return Conversions.angleToDistance(angle, MK4IModuleConstants.WHEEL_DIAMETER_METERS);
    }


    @Override
    public void stop() {
        mk4iModuleActions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        mk4iModuleActions.setBrake(brake);
    }

    @Override
    public void resetByEncoder() {
        mk4iModuleActions.resetSteerAngle(mk4iModuleStatus.getEncoderAbsolutePosition(true));
    }


    @Override
    public void runSteerMotorByVoltage(double voltage) {
        mk4iModuleActions.setTargetSteerVoltage(voltage);
    }

    @Override
    public void runDriveMotorByVoltage(double voltage) {
        mk4iModuleActions.setTargetDriveVoltage(voltage);
    }


    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                mk4iModuleStatus.getSteerMotorLatencyVelocity(true),
                MK4IModuleConstants.COUPLING_RATIO,
                MK4IModuleConstants.MAX_SPEED_PER_SECOND,
                MK4IModuleConstants.WHEEL_DIAMETER_METERS,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        mk4iModuleActions.setTargetDriveVoltage(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(
                targetVelocityMetersPerSecond,
                MK4IModuleConstants.WHEEL_DIAMETER_METERS
        );
        double optimizedVelocityRevolutionsPerSecond = ModuleUtils.removeCouplingFromRevolutions(
                targetVelocityPerSecond,
                mk4iModuleStatus.getSteerMotorLatencyVelocity(true),
                MK4IModuleConstants.COUPLING_RATIO
        );
        mk4iModuleActions.setTargetClosedLoopVelocity(optimizedVelocityRevolutionsPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        mk4iModuleActions.setTargetAngle(angle);
    }


    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        inputs.getEncoderInputs().isConnected = mk4iModuleStatus.refreshEncoderSignals().isOK();
        inputs.getEncoderInputs().angle = mk4iModuleStatus.getEncoderAbsolutePosition(false);
        inputs.getEncoderInputs().velocity = mk4iModuleStatus.getEncoderVelocity(false);
        inputs.getEncoderInputs().voltage = mk4iModuleStatus.getEncoderVoltageSignal(false).getValue();

        inputs.getSteerMotorInputs().isConnected = mk4iModuleStatus.refreshSteerMotorSignals().isOK();
        inputs.getSteerMotorInputs().angle = mk4iModuleStatus.getSteerMotorLatencyPosition(false);
        inputs.getSteerMotorInputs().velocity = mk4iModuleStatus.getSteerMotorLatencyVelocity(false);
        inputs.getSteerMotorInputs().acceleration = mk4iModuleStatus.getSteerMotorAcceleration(false);
        inputs.getSteerMotorInputs().voltage = mk4iModuleStatus.getSteerMotorVoltageSignal(false).getValue();
        inputs.getSteerMotorInputs().angleOdometrySamples = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();

        inputs.getDriveMotorInputs().isConnected = mk4iModuleStatus.refreshDriveMotorSignals().isOK();
        inputs.getDriveMotorInputs().angle = mk4iModuleStatus.getDriveMotorLatencyPosition(false);
        inputs.getDriveMotorInputs().velocity = mk4iModuleStatus.getDriveMotorLatencyVelocity(false);
        inputs.getDriveMotorInputs().acceleration = mk4iModuleStatus.getDriveMotorAcceleration(false);
        inputs.getDriveMotorInputs().current = mk4iModuleStatus.getDriveMotorStatorCurrentSignal(false).getValue();
        inputs.getDriveMotorInputs().voltage = mk4iModuleStatus.getDriveMotorVoltageSignal(false).getValue();
        inputs.getDriveMotorInputs().distanceMeters = toDriveMeters(inputs.getDriveMotorInputs().angle);
        inputs.getDriveMotorInputs().velocityMeters = toDriveMeters(inputs.getDriveMotorInputs().velocity);
        inputs.getDriveMotorInputs().distanceMetersOdometrySamples = drivePositionQueue.stream().mapToDouble(this::toDriveMeters).toArray();
        drivePositionQueue.clear();
    }

}
