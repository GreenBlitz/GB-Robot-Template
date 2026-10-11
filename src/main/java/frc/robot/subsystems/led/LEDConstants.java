package frc.robot.subsystems.led;

public record LEDConstants(
	int portNum,
	int numOfLEDsInStrip,
	double secondsBetweenBlinks,
	double ledAmountPerMeter,
	double scrollSpeedMetersPerSecond,
	int saturation,
	int value
) {}
