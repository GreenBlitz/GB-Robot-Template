package frc.utils.logger;


@FunctionalInterface
public interface ILogger<T> {

	void log(String logPath, T data);

}
