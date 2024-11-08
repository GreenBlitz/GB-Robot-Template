package frc.robot.hardware.interfaces;

public interface IRequest<T> {

	IRequest<T> withSetPoint(T setPoint);

	T getSetPoint();

}
