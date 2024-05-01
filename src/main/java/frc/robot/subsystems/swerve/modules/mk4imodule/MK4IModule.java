package frc.robot.subsystems.swerve.modules.mk4imodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.utils.Conversions;

import java.util.Queue;

public class MK4IModule implements IModule {

    private final MK4IModuleStatus mk4IModuleStatus;

    private final MK4IModuleActions mk4IModuleActions;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    public MK4IModule(ModuleUtils.ModuleName moduleName) {
        MK4IModuleConfigObject moduleConfigObject = getModuleConfigObject(moduleName);
        MK4IModuleRecords.MK4IModuleMotors motors = moduleConfigObject.getMotors();

        this.mk4IModuleStatus = new MK4IModuleStatus(moduleConfigObject.getModuleSignals());
        this.mk4IModuleActions = new MK4IModuleActions(motors);

        this.steerPositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(
                motors.steerMotor(),
                mk4IModuleStatus.getSteerMotorPositionSignal(false)
        );
        this.drivePositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(
                motors.driveMotor(),
                mk4IModuleStatus.getDriveMotorLatencyPosition(false)
        );
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
        mk4IModuleActions.resetSteerAngle(mk4IModuleStatus.getSteerEncoderAbsolutePosition(true));
    }


    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                ModuleConstants.WHEEL_DIAMETER_METERS,
                mk4IModuleStatus.getSteerMotorLatencyVelocity(true),
                MK4IModuleConstants.COUPLING_RATIO,
                ModuleConstants.MAX_SPEED_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        mk4IModuleActions.setTargetOpenLoopVelocity(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        double optimizedVelocityRevolutionsPerSecond = ModuleUtils.removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
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
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        mk4IModuleStatus.refreshAllSignals();

        inputs.steerEncoderAngleDegrees = mk4IModuleStatus.getSteerEncoderAbsolutePosition(false).getDegrees();
        inputs.steerEncoderVelocity = mk4IModuleStatus.getSteerEncoderVelocitySignal(false).getValue();
        inputs.steerEncoderVoltage = mk4IModuleStatus.getSteerEncoderVoltageSignal(false).getValue();

        inputs.steerMotorAngle = mk4IModuleStatus.getSteerMotorLatencyPosition(false);
        inputs.steerMotorVelocity = mk4IModuleStatus.getSteerMotorLatencyVelocity(false);
        inputs.steerMotorVoltage = mk4IModuleStatus.getSteerMotorVoltageSignal(false).getValue();

        inputs.driveDistance = mk4IModuleStatus.getDriveMotorLatencyPosition(false);
        inputs.driveVelocityPerSecond = mk4IModuleStatus.getDriveMotorLatencyVelocity(false);
        inputs.driveCurrent = mk4IModuleStatus.getDriveMotorStatorCurrentSignal(false).getValue();
        inputs.driveVoltage = mk4IModuleStatus.getDriveMotorVoltageSignal(false).getValue();

        inputs.odometryUpdatesSteerAngleDegrees =
                steerPositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray(); //Todo - put in Rot2D
        inputs.odometryUpdatesDriveDistanceDegrees =
                drivePositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray(); //Todo - put in Rot2D

        steerPositionQueue.clear();
        drivePositionQueue.clear();
    }

}
