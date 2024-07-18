package frc.robot.subsystems.swerve.modules.mk4i;

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

public class MK4IModule implements IModule {

    private final MK4IModuleStatus mk4iModuleStatus;
    private final MK4IModuleActions mk4iModuleActions;

    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    private Rotation2d startingSteerAngle;

    public MK4IModule(MK4IModuleConfigObject moduleConfigObject) {
        MK4IModuleRecords.MK4IModuleMotors motors = moduleConfigObject.getMotors();

        this.mk4iModuleStatus = new MK4IModuleStatus(moduleConfigObject.getModuleSignals());
        this.mk4iModuleActions = new MK4IModuleActions(motors);

        this.steerPositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                motors.steerMotor(),
                mk4iModuleStatus.getSteerMotorPositionSignal(false),
                mk4iModuleStatus.getSteerMotorVelocitySignal(false)
        );
        this.drivePositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                motors.driveMotor(),
                mk4iModuleStatus.getDriveMotorPositionSignal(false),
                mk4iModuleStatus.getDriveMotorVelocitySignal(false)
        );

        this.startingSteerAngle = mk4iModuleStatus.getEncoderAbsolutePosition(true);
        mk4iModuleActions.resetDriveAngle(new Rotation2d());
    }

    private double toDriveMeters(Rotation2d angle) {
        return Conversions.angleToDistance(angle, MK4IModuleConstants.WHEEL_DIAMETER_METERS);
    }

    private Rotation2d getUncoupledAngle(Rotation2d driveCoupledAngle, Rotation2d steerAngle){
        return ModuleUtils.getUncoupledAngle(driveCoupledAngle, steerAngle, MK4IModuleConstants.COUPLING_RATIO);
    }


    @Override
    public void stop() {
        mk4iModuleActions.stop();
    }

    @Override
    public void setBrake(boolean brake) {
        mk4iModuleActions.setBrake(brake);
    }

    @Override
    public void resetByEncoder() {
        startingSteerAngle = mk4iModuleStatus.getEncoderAbsolutePosition(true);
        mk4iModuleActions.resetSteerAngle(startingSteerAngle);
    }


    @Override
    public void runSteerMotorByVoltage(double voltage) {
        mk4iModuleActions.setTargetSteerVoltage(voltage);
    }

    @Override
    public void runDriveMotorByVoltage(double voltage) {
        mk4iModuleActions.setTargetDriveVoltage(voltage);
    }


    @Override
    public void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        double voltage = ModuleUtils.velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                mk4iModuleStatus.getSteerMotorLatencyVelocity(true),
                MK4IModuleConstants.COUPLING_RATIO,
                MK4IModuleConstants.MAX_SPEED_PER_SECOND,
                MK4IModuleConstants.WHEEL_DIAMETER_METERS,
                ModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        mk4iModuleActions.setTargetDriveVoltage(voltage);
    }

    @Override
    public void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        Rotation2d targetVelocityPerSecond = Conversions.distanceToAngle(
                targetVelocityMetersPerSecond,
                MK4IModuleConstants.WHEEL_DIAMETER_METERS
        );
        Rotation2d optimizedVelocityPerSecond = ModuleUtils.getCoupledAngle(
                targetVelocityPerSecond,
                mk4iModuleStatus.getSteerMotorLatencyVelocity(true),
                MK4IModuleConstants.COUPLING_RATIO
        );
        mk4iModuleActions.setTargetClosedLoopVelocity(optimizedVelocityPerSecond);
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        mk4iModuleActions.setTargetAngle(angle);
    }


    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        EncoderInputsAutoLogged encoderInputs = inputs.getEncoderInputs();
        encoderInputs.isConnected = mk4iModuleStatus.refreshEncoderSignals().isOK();
        encoderInputs.angle = mk4iModuleStatus.getEncoderAbsolutePosition(false);
        encoderInputs.velocity = mk4iModuleStatus.getEncoderVelocity(false);
        encoderInputs.voltage = mk4iModuleStatus.getEncoderVoltageSignal(false).getValue();

        SteerMotorInputsAutoLogged steerInputs = inputs.getSteerMotorInputs();
        steerInputs.isConnected = mk4iModuleStatus.refreshSteerMotorSignals().isOK();
        steerInputs.angle = mk4iModuleStatus.getSteerMotorLatencyPosition(false);
        steerInputs.velocity = mk4iModuleStatus.getSteerMotorLatencyVelocity(false);
        steerInputs.acceleration = mk4iModuleStatus.getSteerMotorAcceleration(false);
        steerInputs.voltage = mk4iModuleStatus.getSteerMotorVoltageSignal(false).getValue();
        steerInputs.angleOdometrySamples = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();

        DriveMotorInputsAutoLogged driveInputs = inputs.getDriveMotorInputs();
        driveInputs.isConnected = mk4iModuleStatus.refreshDriveMotorSignals().isOK();

        final Rotation2d driveAngle = mk4iModuleStatus.getDriveMotorLatencyPosition(false);
        final Rotation2d steerAngle = Rotation2d.fromRotations(steerInputs.angle.getRotations() - startingSteerAngle.getRotations());
        driveInputs.angle = getUncoupledAngle(driveAngle, steerAngle);

        final Rotation2d driveVelocity = mk4iModuleStatus.getDriveMotorLatencyVelocity(false);
        final Rotation2d steerVelocity = steerInputs.velocity;
        driveInputs.velocity = getUncoupledAngle(driveVelocity, steerVelocity);

        final Rotation2d driveAcceleration = mk4iModuleStatus.getDriveMotorAcceleration(false);
        final Rotation2d steerAcceleration = steerInputs.acceleration;
        driveInputs.acceleration = getUncoupledAngle(driveAcceleration, steerAcceleration);

        driveInputs.current = mk4iModuleStatus.getDriveMotorStatorCurrentSignal(false).getValue();
        driveInputs.voltage = mk4iModuleStatus.getDriveMotorVoltageSignal(false).getValue();
        driveInputs.distanceMeters = toDriveMeters(inputs.getDriveMotorInputs().angle);
        driveInputs.velocityMeters = toDriveMeters(inputs.getDriveMotorInputs().velocity);

        final Rotation2d[] drivePositions = drivePositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        for (int i = 0; i < drivePositions.length; i++){
            Rotation2d steerDelta = Rotation2d.fromRotations(steerInputs.angleOdometrySamples[i].getRotations() - startingSteerAngle.getRotations());
            drivePositions[i] = getUncoupledAngle(drivePositions[i], steerDelta);
        }
        driveInputs.distanceMetersOdometrySamples = Arrays.stream(drivePositions).mapToDouble(this::toDriveMeters).toArray();
        drivePositionQueue.clear();
    }

}
