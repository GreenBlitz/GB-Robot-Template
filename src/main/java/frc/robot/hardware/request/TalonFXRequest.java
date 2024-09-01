package frc.robot.hardware.request;

import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.function.Consumer;

public class TalonFXRequest implements IRequest{

    private final Consumer<Rotation2d> withSetPoint;
    private final ControlRequest request;

    public TalonFXRequest(ControlRequest request, Consumer<Rotation2d> withSetPoint) {
        this.request = request;
        this.withSetPoint = withSetPoint;
    }

    @Override
    public void withSetPoint(Rotation2d setPoint) {
        withSetPoint.accept(setPoint);
    }

    public ControlRequest getRequest() {
        return request;
    }

}
