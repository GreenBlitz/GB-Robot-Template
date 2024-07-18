package frc.robot.subsystems.swerve.modules.talonfx;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.Conversions;

import java.util.Queue;

public class TalonFXModule implements IModule {

    private final TalonFXModuleStatus talonFXModuleStatus;
    private final TalonFXModuleActions talonFXModuleActions;
    private final TalonFXModuleConstants talonFXModuleConstants;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    public TalonFXModule(TalonFXModuleConstants talonFXModuleConstants) {
        this.talonFXModuleConstants = talonFXModuleConstants;
        this.talonFXModuleStatus = new TalonFXModuleStatus(talonFXModuleConstants.getSignals());
        this.talonFXModuleActions = new TalonFXModuleActions(talonFXModuleConstants);

        this.steerPositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                talonFXModuleConstants.getSteerMotor(),
                talonFXModuleStatus.getSteerMotorPositionSignal(false),
                talonFXModuleStatus.getSteerMotorVelocitySignal(false)
        );
        this.drivePositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                talonFXModuleConstants.getDriveMotor(),
                talonFXModuleStatus.getDriveMotorPositionSignal(false),
                talonFXModuleStatus.getDriveMotorVelocitySignal(false)
        );

        talonFXModuleActions.resetDriveAngle(new Rotation2d());
    }

    private double toDriveMeters(double rotations) {
        return toDriveMeters(Rotation2d.fromRotations(rotations));
    }

    private double toDriveMeters(Rotation2d angle) {
        return Conversions.angleToDistance(angle, talonFXModuleConstants.getWheelDiameterMeters());
    }


    @Override
    public void stop() {
        talonFXModuleActions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        talonFXModuleActions.setBrake(brake);
    }

    @Override
    public void resetByEncoder() {
        talonFXModuleActions.resetSteerAngle(talonFXModuleStatus.getEncoderPosition(true));
    }


    @Override
    public void runSteerMotorByVoltage(double voltage) {
        talonFXModuleActions.setTargetSteerVoltage(voltage);
    }

    @Override
    public void runDriveMotorByVoltage(double voltage) {
        talonFXModuleActions.setTargetDriveVoltage(voltage);
    }


    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                talonFXModuleStatus.getSteerMotorLatencyVelocity(true),
                talonFXModuleConstants.getCouplingRatio(),
                talonFXModuleConstants.getVelocityAt12VoltsPerSecond(),
                talonFXModuleConstants.getWheelDiameterMeters(),
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        talonFXModuleActions.setTargetDriveVoltage(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(
                targetVelocityMetersPerSecond,
                talonFXModuleConstants.getWheelDiameterMeters()
        );
        Rotation2d optimizedVelocityPerSecond = ModuleUtils.getUncoupledAngle(
                targetVelocityPerSecond,
                talonFXModuleStatus.getSteerMotorLatencyVelocity(true),
                talonFXModuleConstants.getCouplingRatio()
        );
        talonFXModuleActions.setTargetClosedLoopVelocity(optimizedVelocityPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        talonFXModuleActions.setTargetAngle(angle);
    }


    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        inputs.getEncoderInputs().isConnected = talonFXModuleStatus.refreshEncoderSignals().isOK();
        inputs.getEncoderInputs().angle = talonFXModuleStatus.getEncoderPosition(false);
        inputs.getEncoderInputs().velocity = talonFXModuleStatus.getEncoderVelocity(false);
        inputs.getEncoderInputs().voltage = talonFXModuleStatus.getEncoderVoltageSignal(false).getValue();

        inputs.getSteerMotorInputs().isConnected = talonFXModuleStatus.refreshSteerMotorSignals().isOK();
        inputs.getSteerMotorInputs().angle = talonFXModuleStatus.getSteerMotorLatencyPosition(false);
        inputs.getSteerMotorInputs().velocity = talonFXModuleStatus.getSteerMotorLatencyVelocity(false);
        inputs.getSteerMotorInputs().acceleration = talonFXModuleStatus.getSteerMotorAcceleration(false);
        inputs.getSteerMotorInputs().voltage = talonFXModuleStatus.getSteerMotorVoltageSignal(false).getValue();
        inputs.getSteerMotorInputs().angleOdometrySamples = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();

        inputs.getDriveMotorInputs().isConnected = talonFXModuleStatus.refreshDriveMotorSignals().isOK();
        inputs.getDriveMotorInputs().angle = talonFXModuleStatus.getDriveMotorLatencyPosition(false);
        inputs.getDriveMotorInputs().velocity = talonFXModuleStatus.getDriveMotorLatencyVelocity(false);
        inputs.getDriveMotorInputs().acceleration = talonFXModuleStatus.getDriveMotorAcceleration(false);
        inputs.getDriveMotorInputs().current = talonFXModuleStatus.getDriveMotorStatorCurrentSignal(false).getValue();
        inputs.getDriveMotorInputs().voltage = talonFXModuleStatus.getDriveMotorVoltageSignal(false).getValue();
        inputs.getDriveMotorInputs().distanceMeters = toDriveMeters(inputs.getDriveMotorInputs().angle);
        inputs.getDriveMotorInputs().velocityMeters = toDriveMeters(inputs.getDriveMotorInputs().velocity);
        inputs.getDriveMotorInputs().distanceMetersOdometrySamples = drivePositionQueue.stream().mapToDouble(this::toDriveMeters).toArray();
        drivePositionQueue.clear();
    }

}
