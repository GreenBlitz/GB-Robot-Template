package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.IRequest;
import frc.robot.hardware.interfaces.InputSignal;

public class Roller extends GBSubsystem{
    private final ControllableMotor roller;

    private final IRequest<Double> setVoltageRequest;
    private final InputSignal<Rotation2d> LatencySignal;
    private final InputSignal<Rotation2d> valocetySignal;
    private final InputSignal<Double> voltageSignal;
    private final InputSignal<Rotation2d> currntSignal;

    public Roller(String logPath, IRequest setPower, ControllableMotor roller, IRequest<Double> setVoltageRequest, InputSignal<Rotation2d> latencySignal, InputSignal<Rotation2d> valocetySignal, InputSignal<Double> voltageSignal, InputSignal<Rotation2d> currntSignal) {
        super(logPath);

        this.roller = roller;
        this.setVoltageRequest = setVoltageRequest;

        LatencySignal = latencySignal;
        this.valocetySignal = valocetySignal;
        this.voltageSignal = voltageSignal;
        this.currntSignal = currntSignal;
    }
    public void setVoltage(Double voltage){
        roller.applyRequest(setVoltageRequest.withSetPoint(voltage));
    }

    public void stop(){
        roller.stop();
    }
}
