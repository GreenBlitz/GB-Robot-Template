package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public record LEDConstants(
	int portNum,
	int numOfLEDsInStrip,
	double secondsBetweenBlinks,
	double ledAmountPerMeter,
	double scrollSpeedMetersPerSecond,
	Color defaultColor,
	int saturation,
	int value
) {}
