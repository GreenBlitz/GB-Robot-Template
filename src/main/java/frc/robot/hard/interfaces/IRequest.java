package frc.robot.hard.interfaces;

public interface IRequest<T> {

	IRequest<T> withSetPoint(T setPoint);

	T getSetPoint();

}
