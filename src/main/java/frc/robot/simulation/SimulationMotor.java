package frc.robot.simulation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.IRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.request.phoenix6.Phoenix6DoubleRequest;
import frc.robot.hardware.signal.InputSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6BothLatencySignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.calibration.sysid.SysIdCalibrator;

import java.util.LinkedList;

public class SimulationMotor extends MotorSimulation implements ControllableMotor {

    private final ConnectedInputAutoLogged connectedInput;

    private TalonFXWrapper motor;

    private TalonFXSimState motorSimulationState;

    protected SimulationMotor(){
        super();
        connectedInput = new ConnectedInputAutoLogged();
        connectedInput.connected = true;
    }

    @Override
    public boolean isConnected() {
        return connectedInput.connected;
    }

    @Override
    public void updateSignals(InputSignal... signals) {
        LinkedList<StatusSignal<Double>> signalsSet = new LinkedList<>();
        for (InputSignal signal : signals) {
            if (signal instanceof Phoenix6SignalBuilder.SignalGetter) {
                signalsSet.add(((Phoenix6SignalBuilder.SignalGetter) signal).getSignal());
                if (signal instanceof Phoenix6BothLatencySignal) {
                    signalsSet.add(((Phoenix6BothLatencySignal) signal).getSignalSlope());
                }
            }
        }

        connectedInput.connected = BaseStatusSignal.refreshAll(signalsSet.toArray(StatusSignal[]::new)).isOK();
    }

    @Override
    public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
        return new SysIdCalibrator.SysIdConfigInfo(new SysIdRoutine.Config(),false);
    }

    @Override
    public void resetPosition(Rotation2d position) {
        motor.setPosition(position.getRotations());
    }

    @Override
    public void applyDoubleRequest(IRequest<Double> request) {
        if (request instanceof Phoenix6DoubleRequest) {
            motor.setControl(((Phoenix6DoubleRequest) request).getControlRequest());
        }
    }

    @Override
    public void applyAngleRequest(IRequest<Rotation2d> request) {
        if (request instanceof Phoenix6AngleRequest) {
            motor.setControl(((Phoenix6AngleRequest) request).getControlRequest());
        }
    }

    @Override
    public void setBrake(boolean brake) {
        NeutralModeValue modeValue = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        motor.setNeutralMode(modeValue);
    }

    @Override
    protected void setInputVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    protected void updateMotor() {
        super.updateSimulation();
    }

    @Override
    public Rotation2d getPosition() {
        return Rotation2d.fromRotations(motor.getPosition().getValue());
    }

    @Override
    public Rotation2d getVelocity() {
        return Rotation2d.fromRotations(motor.getVelocity().getValue());
    }

}
