package frc.robot.subsystems.swerve.mk4iswerve.mk4imodule;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.ModuleUtils;
import frc.robot.subsystems.swerve.swerveinterface.IModule;
import frc.robot.subsystems.swerve.swerveinterface.ModuleInputsAutoLogged;

import java.util.Queue;

public class MK4IModule implements IModule {

    private final MK4IData mk4IData;
    private final MK4IActions mk4IActions;
    private Queue<Double> steerPositionQueue, drivePositionQueue;


    public MK4IModule(ModuleUtils.ModuleName moduleName) {
        MK4IModuleConfigObject moduleConfigObject = getModuleConfigObject(moduleName);
        this.mk4IData = new MK4IData(moduleConfigObject);
        this.mk4IActions = new MK4IActions(moduleConfigObject.getDriveMotor(), moduleConfigObject.getSteerMotor(), mk4IData);

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
        mk4IActions.setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        mk4IActions.setTargetClosedLoopVelocity(targetVelocityMetersPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        mk4IActions.setTargetAngle(angle);
    }
    
    @Override
    public void resetByEncoder() {
        mk4IActions.resetByEncoder();
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
