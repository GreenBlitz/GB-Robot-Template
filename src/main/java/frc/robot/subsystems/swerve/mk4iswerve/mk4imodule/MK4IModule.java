package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleConstants;
import frc.robot.subsystems.swerve.ModuleUtils;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;

public class MK4IModule implements IModule {

    private final MK4IModuleStatus mk4IModuleData;

    private final MK4IModuleActions mk4IModuleActions;

    public MK4IModule(ModuleUtils.ModuleName moduleName) {
        MK4IModuleConfigObject moduleConfigObject = getModuleConfigObject(moduleName);

        this.mk4IModuleData = new MK4IModuleStatus(moduleConfigObject.getModuleSignals());
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
                mk4IModuleData.getSteerMotorVelocity().getRotations(),
                MK4IModuleConstants.COUPLING_RATIO,
                ModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        mk4IModuleActions.setTargetOpenLoopVelocity(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = ModuleUtils.removeCouplingFromRevolutions(targetVelocityMetersPerSecond,
                mk4IModuleData.getSteerMotorVelocity(),
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
        mk4IModuleActions.resetSteerAngle(mk4IModuleData.getEncoderAbsolutePosition());
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
        inputs.steerEncoderAngleDegrees = mk4IModuleData.getEncoderAbsolutePosition().getDegrees();
        inputs.steerEncoderVelocity = mk4IModuleData.getSteerEncoderVelocitySignal().getValue();
        inputs.steerEncoderVoltage = mk4IModuleData.getSteerEncoderVoltageSignal().getValue();

        inputs.steerAngleDegrees = mk4IModuleData.getSteerMotorPosition().getDegrees();
        inputs.steerVoltage = mk4IModuleData.getSteerVoltageSignal().getValue();
        inputs.steerVelocity = mk4IModuleData.getSteerMotorVelocity().getRotations();

        inputs.driveDistanceMeters = mk4IModuleData.getDriveMotorPositionInMeters();
        inputs.driveVelocityMetersPerSecond = mk4IModuleData.getDriveMotorVelocityInMeters();
        inputs.driveCurrent = mk4IModuleData.getDriveStatorCurrentSignal().getValue();
        inputs.driveVoltage = mk4IModuleData.getDriveVoltageSignal().getValue();
    }

}
