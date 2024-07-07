package frc.robot.subsystems.swerve.modules.mk4i;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;

import java.util.Queue;

public class MK4IModule implements IModule {

    private final MK4IModuleStatus mk4IModuleStatus;
    private final MK4IModuleActions mk4IModuleActions;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    public MK4IModule(MK4IModuleConfigObject moduleConfigObject) {
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

        mk4IModuleActions.resetDriveAngle(new Rotation2d());
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
        mk4IModuleActions.resetSteerAngle(mk4IModuleStatus.getEncoderAbsolutePosition(true));
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
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(// todo: test
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


    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        inputs.getEncoderInputs().isConnected = mk4IModuleStatus.refreshEncoderSignals().isOK();
        inputs.getEncoderInputs().angle = mk4IModuleStatus.getEncoderAbsolutePosition(false);
        inputs.getEncoderInputs().velocity = mk4IModuleStatus.getEncoderVelocity(false);
        inputs.getEncoderInputs().voltage = mk4IModuleStatus.getEncoderVoltageSignal(false).getValue();

        inputs.getSteerMotorInputs().isConnected = mk4IModuleStatus.refreshSteerMotorSignals().isOK();
        inputs.getSteerMotorInputs().angle = mk4IModuleStatus.getSteerMotorLatencyPosition(false);
        inputs.getSteerMotorInputs().velocity = mk4IModuleStatus.getSteerMotorLatencyVelocity(false);
        inputs.getSteerMotorInputs().acceleration = mk4IModuleStatus.getSteerMotorAcceleration(false);
        inputs.getSteerMotorInputs().voltage = mk4IModuleStatus.getSteerMotorVoltageSignal(false).getValue();
        inputs.getSteerMotorInputs().angleOdometrySamples = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();

        inputs.getDriveMotorInputs().isConnected = mk4IModuleStatus.refreshDriveMotorSignals().isOK();
        inputs.getDriveMotorInputs().angle = mk4IModuleStatus.getDriveMotorLatencyPosition(false);
        inputs.getDriveMotorInputs().velocity = mk4IModuleStatus.getDriveMotorLatencyVelocity(false);
        inputs.getDriveMotorInputs().acceleration = mk4IModuleStatus.getDriveMotorAcceleration(false);
        inputs.getDriveMotorInputs().current = mk4IModuleStatus.getDriveMotorStatorCurrentSignal(false).getValue();
        inputs.getDriveMotorInputs().voltage = mk4IModuleStatus.getDriveMotorVoltageSignal(false).getValue();
        inputs.getDriveMotorInputs().distanceOdometrySamples = drivePositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        drivePositionQueue.clear();
    }

}
