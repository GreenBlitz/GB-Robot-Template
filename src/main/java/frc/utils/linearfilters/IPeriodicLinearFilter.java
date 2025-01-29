package frc.utils.linearfilters;


public interface IPeriodicLinearFilter {

	void hardReset();

	void log(String parentLogPath);

	void update();

}
