package frc.robot.subsystems.swerve.modules.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.goodconstants.module.SimulationModuleConstantsObject;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
import frc.utils.Conversions;
import org.littletonrobotics.junction.Logger;

public class SimulationModule implements IModule {

    private final ModuleUtils.ModuleName moduleName;
    private final SimulationModuleActions simulationModuleActions;
    private final SimulationModuleStatus simulationModuleStatus;
    private final SimulationModuleConstantsObject constants;

    public SimulationModule(ModuleUtils.ModuleName moduleName, SimulationModuleConstantsObject constants) {
        this.moduleName = moduleName;
        this.constants = constants;
        this.simulationModuleActions = new SimulationModuleActions(constants);
        this.simulationModuleStatus = new SimulationModuleStatus(constants.moduleConfigObject());
    }


    private double toDriveMeters(Rotation2d angle) {
        return Conversions.angleToDistance(angle, constants.wheelDiameter());
    }


    @Override
    public void stop() {
        simulationModuleActions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "driveMotorVoltage", simulationModuleStatus.getDriveVoltage());
    }

    @Override
    public void resetByEncoder() {
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "tried to reset by encoder");
    }


    @Override
    public void runSteerMotorByVoltage(double voltage) {
        simulationModuleActions.setTargetSteerVoltage(voltage);
    }

    @Override
    public void runDriveMotorByVoltage(double voltage) {
        simulationModuleActions.setTargetDriveVoltage(voltage);
    }


    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                simulationModuleStatus.getSteerVelocity(),
                0,
                constants.maxVelocityPerSecond(),
                constants.wheelDiameter(),
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        Logger.recordOutput(ModuleUtils.getLoggingPath(moduleName) + "driveMotorVoltage", simulationModuleStatus.getDriveVoltage());
        runDriveMotorByVoltage(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        setTargetOpenLoopVelocity(targetVelocityMetersPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        simulationModuleActions.setTargetAngle(angle);
    }


    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        inputs.getSteerMotorInputs().angle = simulationModuleStatus.getSteerPosition();
        inputs.getSteerMotorInputs().velocity = simulationModuleStatus.getSteerVelocity();
        inputs.getSteerMotorInputs().voltage = simulationModuleStatus.getSteerVoltage();
        inputs.getSteerMotorInputs().angleOdometrySamples = new Rotation2d[]{inputs.getSteerMotorInputs().angle};

        inputs.getDriveMotorInputs().angle = simulationModuleStatus.getDrivePositionAngle();
        inputs.getDriveMotorInputs().velocity = simulationModuleStatus.getDriveVelocityAnglePerSecond();
        inputs.getDriveMotorInputs().current = simulationModuleStatus.getDriveCurrent();
        inputs.getDriveMotorInputs().voltage = simulationModuleStatus.getDriveVoltage();
        inputs.getDriveMotorInputs().distanceMeters = toDriveMeters(inputs.getDriveMotorInputs().angle);
        inputs.getDriveMotorInputs().velocityMeters = toDriveMeters(inputs.getDriveMotorInputs().velocity);
        inputs.getDriveMotorInputs().distanceMetersOdometrySamples = new double[]{inputs.getDriveMotorInputs().distanceMeters};
    }

}
