package frc.robot.subsystems.swerve.modules.check.drive.talonfx;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.check.drive.DriveMotorInputsAutoLogged;
import frc.robot.subsystems.swerve.modules.check.drive.IDrive;
import frc.robot.subsystems.swerve.modules.inputs.ModuleInputsContainer;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.Arrays;
import java.util.Queue;

public class TalonFXDrive implements IDrive {

    private final TalonFXWrapper driveMotor;
    private final TalonFXDriveSignals signals;

    private final VelocityVoltage velocityVoltageRequest;
    private final VoltageOut voltageRequest;

    private final Queue<Double> drivePositionQueue;

    public TalonFXDrive(TalonFXDriveConstants constants){
        this.driveMotor = constants.getDriveMotor();
        this.signals = constants.getSignals();

        this.velocityVoltageRequest = new VelocityVoltage(0).withEnableFOC(constants.getEnableFOC());
        this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOC());

        this.drivePositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                driveMotor,
                signals.drivePositionSignal(),
                signals.driveVelocitySignal()
        );

        driveMotor.setPosition(0);
    }


    @Override
    public void setBrake(boolean brake) {
        NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
    }

    @Override
    public void stop() {
        driveMotor.stopMotor();
    }

    @Override
    public void runMotorByVoltage(double voltage) {
        driveMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetClosedLoopVelocity(Rotation2d velocityPerSecond) {
        driveMotor.setControl(velocityVoltageRequest.withVelocity(velocityPerSecond.getRotations()));
    }

    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
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
