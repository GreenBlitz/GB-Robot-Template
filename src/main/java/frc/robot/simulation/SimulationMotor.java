package frc.robot.simulation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.ConnectedInputAutoLogged;
import frc.robot.hardware.motor.ControllableMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.IRequest;
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

        connectedInput.connected = BaseStatusSignal.isAllGood(signalsSet.toArray(StatusSignal[]::new));
        BaseStatusSignal.refreshAll(signalsSet.toArray(StatusSignal[]::new));
    }

    @Override
    public SysIdCalibrator.SysIdConfigInfo getSysidConfigInfo() {
        return null;
    }

    @Override
    public void resetPosition(Rotation2d position) {

    }

    @Override
    public void applyDoubleRequest(IRequest<Double> request) {

    }

    @Override
    public void applyAngleRequest(IRequest<Rotation2d> request) {

    }

    @Override
    public void setBrake(boolean brake) {

    }

    @Override
    protected void setInputVoltage(double voltage) {

    }

    @Override
    protected void updateMotor() {

    }

    @Override
    public Rotation2d getPosition() {
        return null;
    }

    @Override
    public Rotation2d getVelocity() {
        return null;
    }
}
