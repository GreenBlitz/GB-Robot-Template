package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.modules.ModuleInputsContainer;
import frc.robot.subsystems.swerve.modules.steer.ISteer;
import frc.robot.subsystems.swerve.modules.steer.SteerInputsAutoLogged;
import frc.robot.subsystems.swerve.odometryThread.PhoenixOdometryThread6328;
import frc.utils.devicewrappers.TalonFXWrapper;

import java.util.Queue;

public class TalonFXSteer implements ISteer {

    private final TalonFXWrapper steerMotor;
    private final TalonFXSteerSignals signals;

    private final PositionVoltage positionVoltageRequest;
    private final VoltageOut voltageRequest;

    private final Queue<Double> steerPositionQueue;

    public TalonFXSteer(TalonFXSteerConstants constants){
        this.steerMotor = constants.getSteerMotor();
        this.signals = constants.getSignals();

        this.positionVoltageRequest = new PositionVoltage(0).withEnableFOC(constants.getEnableFOC());
        this.voltageRequest = new VoltageOut(0).withEnableFOC(constants.getEnableFOC());

        this.steerPositionQueue = PhoenixOdometryThread6328.getInstance().registerLatencySignal(
                steerMotor,
                signals.positionSignal(),
                signals.velocitySignal()
        );
    }

    @Override
    public void setBrake(boolean brake) {
        NeutralModeValue neutralModeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        steerMotor.setNeutralMode(neutralModeValue);
    }

    @Override
    public void resetToAngle(Rotation2d angle) {
        steerMotor.setPosition(angle.getRotations());
    }


    @Override
    public void stop() {
        steerMotor.stopMotor();
    }

    @Override
    public void runMotorByVoltage(double voltage) {
        steerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetAngle(Rotation2d angle) {
        steerMotor.setControl(positionVoltageRequest.withPosition(angle.getRotations()));
    }


    @Override
    public void updateInputs(ModuleInputsContainer inputs) {
        SteerInputsAutoLogged steerInputs = inputs.getSteerMotorInputs();
        steerInputs.isConnected = BaseStatusSignal.refreshAll(
                signals.positionSignal(),
                signals.velocitySignal(),
                signals.accelerationSignal(),
                signals.voltageSignal()
        ).isOK();
        steerInputs.angle = Rotation2d.fromRotations(steerMotor.getLatencyCompensatedPosition());
        steerInputs.velocity = Rotation2d.fromRotations(steerMotor.getLatencyCompensatedVelocity());
        steerInputs.acceleration = Rotation2d.fromRotations(signals.accelerationSignal().getValue());
        steerInputs.voltage = signals.voltageSignal().getValue();
        steerInputs.angleOdometrySamples = steerPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);
        steerPositionQueue.clear();
    }

}
