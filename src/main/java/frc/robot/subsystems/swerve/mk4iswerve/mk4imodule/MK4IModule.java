package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleConstants;
import frc.robot.subsystems.swerve.ModuleUtils;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;

public class MK4IModule implements IModule {

    private final MK4IModuleStatus mk4IModuleStatus;

    private final MK4IModuleActions mk4IModuleActions;

    public MK4IModule(ModuleUtils.ModuleName moduleName) {
        final MK4IModuleConfigObject moduleConfigObject = getModuleConfigObject(moduleName);

        this.mk4IModuleStatus = new MK4IModuleStatus(moduleConfigObject.getModuleSignals());
        this.mk4IModuleActions = new MK4IModuleActions(moduleConfigObject.getMotors());
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
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = ModuleUtils.velocityToOpenLoopVoltage(targetVelocityMetersPerSecond,
                ModuleConstants.WHEEL_DIAMETER_METERS,
                mk4IModuleStatus.getSteerMotorVelocity().getRotations(),
                MK4IModuleConstants.COUPLING_RATIO,
                ModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        mk4IModuleActions.setTargetOpenLoopVelocity(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond =
                ModuleUtils.removeCouplingFromRevolutions(targetVelocityMetersPerSecond,
                mk4IModuleStatus.getSteerMotorVelocity(),
                MK4IModuleConstants.COUPLING_RATIO
        );
        mk4IModuleActions.setTargetClosedLoopVelocity(optimizedVelocityRevolutionsPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        mk4IModuleActions.setTargetAngle(angle);
    }

    @Override
    public void resetByEncoder() {
        mk4IModuleActions.resetSteerAngle(mk4IModuleStatus.getEncoderAbsolutePosition());
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
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        inputs.steerEncoderAngleDegrees = mk4IModuleStatus.getEncoderAbsolutePosition().getDegrees();
        inputs.steerEncoderVelocity = mk4IModuleStatus.getSteerEncoderVelocitySignal().getValue();
        inputs.steerEncoderVoltage = mk4IModuleStatus.getSteerEncoderVoltageSignal().getValue();

        inputs.steerAngleDegrees = mk4IModuleStatus.getSteerMotorPosition().getDegrees();
        inputs.steerVoltage = mk4IModuleStatus.getSteerVoltageSignal().getValue();
        inputs.steerVelocity = mk4IModuleStatus.getSteerMotorVelocity().getRotations();

        inputs.driveDistanceMeters = mk4IModuleStatus.getDriveMotorPositionInMeters();
        inputs.driveVelocityMetersPerSecond = mk4IModuleStatus.getDriveMotorVelocityInMeters();
        inputs.driveCurrent = mk4IModuleStatus.getDriveStatorCurrentSignal().getValue();
        inputs.driveVoltage = mk4IModuleStatus.getDriveVoltageSignal().getValue();
    }

}
