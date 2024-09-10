package frc.robot.hardware.request.phoenix6;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.hardware.request.IRequest;

import java.util.function.Consumer;

public class Phoenix6DoubleRequest implements IRequest<Double> {

    private final Consumer<Double> withSetPoint;
    private final ControlRequest controlRequest;

    private Phoenix6DoubleRequest(ControlRequest controlRequest, Consumer<Double> withSetPoint) {
        this.withSetPoint = withSetPoint;
        this.controlRequest = controlRequest;
    }

    public Phoenix6DoubleRequest(VoltageOut voltageOut) {
        this(voltageOut, voltageOut::withOutput);
    }

    public Phoenix6DoubleRequest(TorqueCurrentFOC torqueCurrentFOC) {
        this(torqueCurrentFOC, torqueCurrentFOC::withOutput);
    }

    @Override
    public Phoenix6DoubleRequest withSetPoint(Double setPoint) {
        withSetPoint.accept(setPoint);
        return this;
    }

    public ControlRequest getControlRequest() {
        return controlRequest;
    }

}
