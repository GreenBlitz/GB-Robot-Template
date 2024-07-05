package frc.robot.subsystems.swerve.modules.mk4i;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.moduleinterface.IModule;
import frc.robot.subsystems.swerve.modules.moduleinterface.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;

import java.util.Queue;
import java.util.concurrent.atomic.AtomicInteger;

public class MK4IModule implements IModule {

    private final MK4IModuleStatus mk4IModuleStatus;
    private final MK4IModuleActions mk4IModuleActions;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    private Rotation2d startSteerAngle;

    public MK4IModule(ModuleUtils.ModuleName moduleName) {
        MK4IModuleConfigObject moduleConfigObject = getModuleConfigObject(moduleName);
        MK4IModuleRecords.MK4IModuleMotors motors = moduleConfigObject.getMotors();

        this.mk4IModuleStatus = new MK4IModuleStatus(moduleConfigObject.getModuleSignals());
        this.mk4IModuleActions = new MK4IModuleActions(motors);

        this.steerPositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                motors.steerMotor(),
                mk4IModuleStatus.getSteerMotorPositionSignal(false),
                mk4IModuleStatus.getSteerMotorVelocitySignal(false)
        );
        this.drivePositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                motors.driveMotor(),
                mk4IModuleStatus.getDriveMotorPositionSignal(false),
                mk4IModuleStatus.getDriveMotorVelocitySignal(false)
        );

        this.startSteerAngle = mk4IModuleStatus.getEncoderAbsolutePosition(true);
        mk4IModuleActions.resetDriveAngle(new Rotation2d()); // for drive distance calculation
    }

    private MK4IModuleConfigObject getModuleConfigObject(ModuleUtils.ModuleName moduleName) {
        return switch (moduleName) {
            case FRONT_LEFT -> MK4IModuleConstants.FRONT_LEFT;
            case FRONT_RIGHT -> MK4IModuleConstants.FRONT_RIGHT;
            case BACK_LEFT -> MK4IModuleConstants.BACK_LEFT;
            case BACK_RIGHT -> MK4IModuleConstants.BACK_RIGHT;
        };
    }


    @Override
    public void stop() {
        mk4IModuleActions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        mk4IModuleActions.setBrake(brake);
    }

    @Override
    public void resetByEncoder() {
        Rotation2d encoderAbsoluteAngle = mk4IModuleStatus.getEncoderAbsolutePosition(true);
        mk4IModuleActions.resetSteerAngle(encoderAbsoluteAngle);
        startSteerAngle = encoderAbsoluteAngle;
    }


    @Override
    public void runSteerMotorByVoltage(double voltage) {
        mk4IModuleActions.setTargetSteerVoltage(voltage);
    }

    @Override
    public void runDriveMotorByVoltage(double voltage) {
        mk4IModuleActions.setTargetDriveVoltage(voltage);
    }


    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                mk4IModuleStatus.getSteerMotorLatencyVelocity(true),
                MK4IModuleConstants.COUPLING_RATIO,
                ModuleConstants.MAX_SPEED_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        mk4IModuleActions.setTargetDriveVoltage(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        Rotation2d targetVelocityPerSecond = ModuleUtils.fromDriveMetersToDriveAngle(targetVelocityMetersPerSecond);
        double optimizedVelocityRevolutionsPerSecond = ModuleUtils.removeCouplingFromRevolutions(
                targetVelocityPerSecond,
                mk4IModuleStatus.getSteerMotorLatencyVelocity(true),
                MK4IModuleConstants.COUPLING_RATIO
        );
        mk4IModuleActions.setTargetClosedLoopVelocity(optimizedVelocityRevolutionsPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        mk4IModuleActions.setTargetAngle(angle);
    }


    // todo: delete and do resistance instead
    private Rotation2d getDriveDistanceWithoutCoupling(double driveDistanceRot, Rotation2d steerAngle) {
        return Rotation2d.fromRotations(
                driveDistanceRot - ((steerAngle.getRotations() - startSteerAngle.getRotations()) * MK4IModuleConstants.COUPLING_RATIO)
        );
    }

    @Override
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        inputs.isEncoderConnected = mk4IModuleStatus.refreshEncoderSignals().isOK();
        inputs.encoderAngle = mk4IModuleStatus.getEncoderAbsolutePosition(false);
        inputs.encoderVelocity = Rotation2d.fromRotations(mk4IModuleStatus.getEncoderVelocitySignal(false).getValue());
        inputs.encoderVoltage = mk4IModuleStatus.getEncoderVoltageSignal(false).getValue();

        inputs.isEncoderConnected = mk4IModuleStatus.refreshSteerMotorSignals().isOK();
        inputs.steerMotorAngle = mk4IModuleStatus.getSteerMotorLatencyPosition(false);
        inputs.steerMotorVelocity = mk4IModuleStatus.getSteerMotorLatencyVelocity(false);
        inputs.steerMotorAcceleration = mk4IModuleStatus.getSteerMotorAcceleration(false);
        inputs.steerMotorVoltage = mk4IModuleStatus.getSteerMotorVoltageSignal(false).getValue();
        inputs.odometrySamplesSteerAngle = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();

        inputs.isDriveMotorConnected = mk4IModuleStatus.refreshDriveMotorSignals().isOK();
        inputs.driveMotorAngleWithoutCoupling = getDriveDistanceWithoutCoupling(// todo: delete and do resistance instead
                mk4IModuleStatus.getDriveMotorLatencyPosition(false).getRotations(),
                inputs.steerMotorAngle
        );
        inputs.driveMotorVelocityWithoutCoupling = getDriveDistanceWithoutCoupling(// todo: delete and do resistance instead
                mk4IModuleStatus.getDriveMotorLatencyVelocity(false).getRotations(),
                inputs.steerMotorVelocity
        );
        inputs.driveMotorAcceleration = mk4IModuleStatus.getDriveMotorAcceleration(false);
        inputs.driveMotorCurrent = mk4IModuleStatus.getDriveMotorStatorCurrentSignal(false).getValue();
        inputs.driveMotorVoltage = mk4IModuleStatus.getDriveMotorVoltageSignal(false).getValue();
        // todo: delete and do resistance instead
        AtomicInteger count = new AtomicInteger();
        inputs.odometrySamplesDriveDistance = drivePositionQueue.stream().map(drive -> getDriveDistanceWithoutCoupling(
                drive,
                inputs.odometrySamplesSteerAngle[count.getAndIncrement()]
        )).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
    }

}
