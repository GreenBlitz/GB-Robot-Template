package frc.utils.linearfilters;


/**
 * This class represents a periodic filter. Filter is an object that takes data over time and reduces it to a single value. A common use case is
 * linear filter.
 *
 * @param <T>
 */
public interface IPeriodicFilter<T> {

	void hardReset();

	void log(String parentLogPath);

	void update();

	T get();

}
