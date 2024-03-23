package frc.robot.subsystems.swerve.falconswerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveModuleInputsAutoLogged;
import frc.utils.Conversions;
import frc.utils.devicewrappers.GBTalonFXPro;

import java.util.Queue;

public class FalconSwerveModule extends SwerveModuleIO {
    private final GBTalonFXPro steerMotor, driveMotor;
    private final FalconSwerveModuleConstants moduleConstants;
    private final Queue<Double> steerPositionQueue, drivePositionQueue;

    private final VelocityTorqueCurrentFOC driveVelocityRequest = new VelocityTorqueCurrentFOC(0);
    private final VoltageOut driveVoltageRequest = new VoltageOut(0).withEnableFOC(FalconSwerveModuleConstants.ENABLE_FOC);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0).withEnableFOC(FalconSwerveModuleConstants.ENABLE_FOC);

    FalconSwerveModule(FalconSwerveModuleConstants moduleConstants, String moduleName) {
        super(moduleName);

        this.steerMotor = moduleConstants.steerMotor;
        this.driveMotor = moduleConstants.driveMotor;
        this.moduleConstants = moduleConstants;
        this.steerPositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(steerMotor, moduleConstants.steerPositionSignal);
        this.drivePositionQueue = TalonFXOdometryThread6328.getInstance().registerSignal(driveMotor, moduleConstants.drivePositionSignal);
    }

    private double getAngleDegrees() {
        return Conversions.revolutionsToDegrees(moduleConstants.steerPositionSignal.getValue());
    }

    private double toDriveDistance(double revolutions) {
        return Conversions.revolutionsToDistance(revolutions, FalconSwerveModuleConstants.WHEEL_DIAMETER_METERS);
    }

    @Override
    protected void setTargetOpenLoopVelocity(double targetVelocityMetersPerSecond) {
        final double voltage = velocityToOpenLoopVoltage(
                targetVelocityMetersPerSecond,
                FalconSwerveModuleConstants.WHEEL_DIAMETER_METERS,
                moduleConstants.steerVelocitySignal.getValue(),
                FalconSwerveModuleConstants.COUPLING_RATIO,
                FalconSwerveModuleConstants.MAX_SPEED_REVOLUTIONS_PER_SECOND,
                FalconSwerveModuleConstants.VOLTAGE_COMPENSATION_SATURATION
        );
        driveMotor.setControl(driveVoltageRequest.withOutput(voltage));
    }

    @Override
    protected void setTargetClosedLoopVelocity(double targetVelocityMetersPerSecond) {
        final double optimizedVelocityRevolutionsPerSecond = removeCouplingFromRevolutions(
                targetVelocityMetersPerSecond,
                Rotation2d.fromDegrees(moduleConstants.steerVelocitySignal.getValue()),
                FalconSwerveModuleConstants.COUPLING_RATIO
        );
        driveMotor.setControl(driveVelocityRequest.withVelocity(optimizedVelocityRevolutionsPerSecond));
    }

    @Override
    protected void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(steerPositionRequest.withPosition(angle.getRotations()));
    }

    @Override
    protected void stop() {
        steerMotor.stopMotor();
        driveMotor.stopMotor();
    }

    @Override
    protected void setBrake(boolean brake) {
        final NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveMotor.setNeutralMode(neutralModeValue);
        steerMotor.setNeutralMode(neutralModeValue);
    }

    private void refreshStatusSignals() {
        BaseStatusSignal.refreshAll(
                moduleConstants.steerPositionSignal,
                moduleConstants.steerVelocitySignal,
                moduleConstants.drivePositionSignal,
                moduleConstants.driveVelocitySignal,
                moduleConstants.driveStatorCurrentSignal
        );
    }

    @Override
    protected void updateInputs(SwerveModuleInputsAutoLogged inputs) {
        refreshStatusSignals();

        inputs.steerAngleDegrees = getAngleDegrees();
        inputs.odometryUpdatesSteerAngleDegrees = steerPositionQueue.stream().mapToDouble(Conversions::revolutionsToDegrees).toArray();
        inputs.steerVoltage = moduleConstants.steerVoltageSignal.getValue();

        inputs.driveDistanceMeters = toDriveDistance(moduleConstants.drivePositionSignal.getValue());
        inputs.odometryUpdatesDriveDistanceMeters = drivePositionQueue.stream().mapToDouble(this::toDriveDistance).toArray();
        inputs.driveVelocityMetersPerSecond = toDriveDistance(moduleConstants.driveVelocitySignal.getValue());
        inputs.driveCurrent = moduleConstants.driveStatorCurrentSignal.getValue();
        inputs.driveVoltage = moduleConstants.driveVoltageSignal.getValue();

        steerPositionQueue.clear();
        drivePositionQueue.clear();
    }
}
