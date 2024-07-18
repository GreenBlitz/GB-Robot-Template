package frc.robot.subsystems.swerve.modules.talonfx;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.IModule;
import frc.robot.subsystems.swerve.modules.ModuleConstants;
import frc.robot.subsystems.swerve.modules.ModuleUtils;
import frc.robot.subsystems.swerve.modules.inputs.DriveMotorInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.inputs.EncoderInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.inputs.SteerMotorInputsAutoLogged;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.Conversions;

import java.util.Arrays;
import java.util.Queue;

public class TalonFXModule implements IModule {

    private final TalonFXModuleStatus talonFXModuleStatus;
    private final TalonFXModuleActions talonFXModuleActions;
    private final TalonFXModuleConstants talonFXModuleConstants;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    private Rotation2d startingSteerAngle;

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

        this.startingSteerAngle = talonFXModuleStatus.getEncoderPosition(true);
        talonFXModuleActions.resetDriveAngle(new Rotation2d());
    }


    private double toDriveMeters(Rotation2d angle) {
        return Conversions.angleToDistance(angle, talonFXModuleConstants.getWheelDiameterMeters());
    }

    private Rotation2d getUncoupledAngle(Rotation2d driveCoupledAngle, Rotation2d steerAngle){
        return ModuleUtils.getUncoupledAngle(driveCoupledAngle, steerAngle, talonFXModuleConstants.getCouplingRatio());
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
        startingSteerAngle = talonFXModuleStatus.getEncoderPosition(true);
        talonFXModuleActions.resetSteerAngle(startingSteerAngle);
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
        Rotation2d optimizedVelocityPerSecond = ModuleUtils.getCoupledAngle(
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
        EncoderInputsAutoLogged encoderInputs = inputs.getEncoderInputs();
        encoderInputs.isConnected = talonFXModuleStatus.refreshEncoderSignals().isOK();
        encoderInputs.angle = talonFXModuleStatus.getEncoderPosition(false);
        encoderInputs.velocity = talonFXModuleStatus.getEncoderVelocity(false);
        encoderInputs.voltage = talonFXModuleStatus.getEncoderVoltageSignal(false).getValue();

        SteerMotorInputsAutoLogged steerInputs = inputs.getSteerMotorInputs();
        steerInputs.isConnected = talonFXModuleStatus.refreshSteerMotorSignals().isOK();
        steerInputs.angle = talonFXModuleStatus.getSteerMotorLatencyPosition(false);
        steerInputs.velocity = talonFXModuleStatus.getSteerMotorLatencyVelocity(false);
        steerInputs.acceleration = talonFXModuleStatus.getSteerMotorAcceleration(false);
        steerInputs.voltage = talonFXModuleStatus.getSteerMotorVoltageSignal(false).getValue();
        steerInputs.angleOdometrySamples = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();

        DriveMotorInputsAutoLogged driveInputs = inputs.getDriveMotorInputs();
        driveInputs.isConnected = talonFXModuleStatus.refreshDriveMotorSignals().isOK();

        final Rotation2d driveAngle = talonFXModuleStatus.getDriveMotorLatencyPosition(false);
        final Rotation2d steerAngle = Rotation2d.fromRotations(steerInputs.angle.getRotations() - startingSteerAngle.getRotations());
        driveInputs.angle = getUncoupledAngle(driveAngle, steerAngle);

        final Rotation2d driveVelocity = talonFXModuleStatus.getDriveMotorLatencyVelocity(false);
        final Rotation2d steerVelocity = steerInputs.velocity;
        driveInputs.velocity = getUncoupledAngle(driveVelocity, steerVelocity);

        final Rotation2d driveAcceleration = talonFXModuleStatus.getDriveMotorAcceleration(false);
        final Rotation2d steerAcceleration = steerInputs.acceleration;
        driveInputs.acceleration = getUncoupledAngle(driveAcceleration, steerAcceleration);

        driveInputs.current = talonFXModuleStatus.getDriveMotorStatorCurrentSignal(false).getValue();
        driveInputs.voltage = talonFXModuleStatus.getDriveMotorVoltageSignal(false).getValue();
        driveInputs.distanceMeters = toDriveMeters(driveInputs.angle);
        driveInputs.velocityMeters = toDriveMeters(driveInputs.velocity);

        final Rotation2d[] drivePositions = drivePositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        for (int i = 0; i < drivePositions.length; i++){
            Rotation2d steerDelta = Rotation2d.fromRotations(steerInputs.angleOdometrySamples[i].getRotations() - startingSteerAngle.getRotations());
            drivePositions[i] = getUncoupledAngle(drivePositions[i], steerDelta);
        }
        driveInputs.distanceMetersOdometrySamples = Arrays.stream(drivePositions).mapToDouble(this::toDriveMeters).toArray();
        drivePositionQueue.clear();
    }

}
