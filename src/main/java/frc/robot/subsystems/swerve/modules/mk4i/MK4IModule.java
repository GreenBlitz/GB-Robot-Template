package frc.robot.subsystems.swerve.modules.mk4i;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
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
    public void updateInputs(ModuleInputsContainer inputs) {
        inputs.getEncoderInputs().isEncoderConnected = mk4IModuleStatus.refreshEncoderSignals().isOK();
        inputs.getEncoderInputs().encoderAngle = mk4IModuleStatus.getEncoderAbsolutePosition(false);
        inputs.getEncoderInputs().encoderVelocity = mk4IModuleStatus.getEncoderVelocity(false);
        inputs.getEncoderInputs().encoderVoltage = mk4IModuleStatus.getEncoderVoltageSignal(false).getValue();

        inputs.getSteerMotorInputs().isSteerMotorConnected = mk4IModuleStatus.refreshSteerMotorSignals().isOK();
        inputs.getSteerMotorInputs().steerMotorAngle = mk4IModuleStatus.getSteerMotorLatencyPosition(false);
        inputs.getSteerMotorInputs().steerMotorVelocity = mk4IModuleStatus.getSteerMotorLatencyVelocity(false);
        inputs.getSteerMotorInputs().steerMotorAcceleration = mk4IModuleStatus.getSteerMotorAcceleration(false);
        inputs.getSteerMotorInputs().steerMotorVoltage = mk4IModuleStatus.getSteerMotorVoltageSignal(false).getValue();
        inputs.getSteerMotorInputs().odometrySamplesSteerAngle = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();

        inputs.getDriveMotorInputs().isDriveMotorConnected = mk4IModuleStatus.refreshDriveMotorSignals().isOK();
        // todo: delete and do resistance instead
        inputs.getDriveMotorInputs().driveMotorAngleWithoutCoupling = getDriveDistanceWithoutCoupling(
                mk4IModuleStatus.getDriveMotorLatencyPosition(false).getRotations(),
                inputs.getSteerMotorInputs().steerMotorAngle
        );
        // todo: delete and do resistance instead
        inputs.getDriveMotorInputs().driveMotorVelocityWithoutCoupling = getDriveDistanceWithoutCoupling(
                mk4IModuleStatus.getDriveMotorLatencyVelocity(false).getRotations(),
                inputs.getSteerMotorInputs().steerMotorVelocity
        );
        inputs.getDriveMotorInputs().driveMotorAcceleration = mk4IModuleStatus.getDriveMotorAcceleration(false);
        inputs.getDriveMotorInputs().driveMotorCurrent = mk4IModuleStatus.getDriveMotorStatorCurrentSignal(false).getValue();
        inputs.getDriveMotorInputs().driveMotorVoltage = mk4IModuleStatus.getDriveMotorVoltageSignal(false).getValue();
        // todo: delete and do resistance instead
        AtomicInteger count = new AtomicInteger();
        inputs.getDriveMotorInputs().odometrySamplesDriveDistance =
                drivePositionQueue.stream().map(drive -> getDriveDistanceWithoutCoupling(
                        drive,
                        inputs.getSteerMotorInputs().odometrySamplesSteerAngle[count.getAndIncrement()]
        )).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
    }

}
