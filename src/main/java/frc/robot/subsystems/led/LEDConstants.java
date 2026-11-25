package frc.robot.subsystems.led;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.util.Color;

public record LEDConstants(
	String LOG_PATH,
	int PORT_NUMBER,
	int NUMBER_OF_LEDS_IN_STRIP,
	double DEFAULT_TIME_BETWEEN_BLINKS_SECOND,
	double NUMBER_OF_LEDS_PER_METER,
	LinearVelocity SCROLL_SPEED_PER_METER_PER_SECOND,
	Color DEFAULT_COLOR,
	int SATURATION,
	int VALUE
) {}
