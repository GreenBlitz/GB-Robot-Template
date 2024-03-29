package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleConstants;
import frc.robot.subsystems.swerve.ModuleUtils;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;

import java.util.Queue;

public class MK4IModule implements IModule {

    private final MK4IModuleData mk4IData;
    private final MK4IModuleActions mk4IActions;
    private Queue<Double> steerPositionQueue, drivePositionQueue;


    public MK4IModule(ModuleUtils.ModuleName moduleName) {
        MK4IModuleConfigObject moduleConfigObject = getModuleConfigObject(moduleName);
        this.mk4IData = new MK4IModuleData(moduleConfigObject);
        this.mk4IActions = new MK4IModuleActions(
                moduleConfigObject.getDriveMotor(),
                moduleConfigObject.getSteerMotor()
        );

//        this.steerPositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(steerMotor, moduleConfigObject.steerPositionSignal);
//        this.drivePositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(driveMotor, moduleConfigObject.drivePositionSignal);
    }

    public MK4IModuleConfigObject getModuleConfigObject(ModuleUtils.ModuleName moduleName){
        return switch (moduleName) {
            case FRONT_LEFT -> MK4IModuleConstants.FRONT_LEFT;
            case FRONT_RIGHT -> MK4IModuleConstants.FRONT_RIGHT;
            case BACK_LEFT -> MK4IModuleConstants.BACK_LEFT;
            case BACK_RIGHT -> MK4IModuleConstants.BACK_RIGHT;
        };
    }

    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                ModuleConstants.WHEEL_DIAMETER_METERS,
                mk4IData.getSteerMotorVelocity().getRotations(),
                MK4IModuleConstants.COUPLING_RATIO,
                ModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        mk4IActions.setTargetOpenLoopVelocity(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = ModuleUtils.removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
                mk4IData.getSteerMotorVelocity(),
                MK4IModuleConstants.COUPLING_RATIO
        );
        mk4IActions.setTargetClosedLoopVelocity(optimizedVelocityRevolutionsPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        mk4IActions.setTargetAngle(angle);
    }
    
    @Override
    public void resetByEncoder() {
        mk4IActions.resetSteerAngle(mk4IData.getEncoderAbsolutePosition());
    }
    
    @Override
    public void stop() {
        mk4IActions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        mk4IActions.setBrake(brake);
    }

    @Override
    public void updateInputs(ModuleInputsAutoLogged inputs) {
        inputs.steerEncoderAngleDegrees = mk4IData.getEncoderAbsolutePosition().getDegrees();
        inputs.steerEncoderVelocity = mk4IData.getSteerEncoderVelocitySignal().getValue();
        inputs.steerEncoderVoltage = mk4IData.getSteerEncoderVoltageSignal().getValue();

        inputs.steerAngleDegrees = mk4IData.getSteerMotorPosition().getDegrees();
//        inputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray();
        inputs.steerVoltage = mk4IData.getSteerVoltageSignal().getValue();
        inputs.steerVelocity = mk4IData.getSteerMotorVelocity().getRotations();

        inputs.driveDistanceMeters = mk4IData.getDriveMotorPositionInMeters();
//        inputs.odometryUpdatesDriveDistanceMeters = drivePositionQueue.stream().mapToDouble(this::toDriveDistance).toArray();
        inputs.driveVelocityMetersPerSecond = mk4IData.getDriveMotorVelocityInMeters();
        inputs.driveCurrent = mk4IData.getDriveStatorCurrentSignal().getValue();
        inputs.driveVoltage = mk4IData.getDriveVoltageSignal().getValue();

//        steerPositionQueue.clear();
//        drivePositionQueue.clear();
    }

}
