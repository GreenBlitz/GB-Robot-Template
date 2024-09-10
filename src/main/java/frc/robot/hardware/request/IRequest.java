package frc.robot.hardware.request;

public interface IRequest<T> {

    IRequest<T> withSetPoint(T setPoint);

}
