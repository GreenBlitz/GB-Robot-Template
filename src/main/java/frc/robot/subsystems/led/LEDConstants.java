package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.util.Color;

public record LEDConstants(
	int PORT_NUMBER,
	int NUMBER_OF_LEDS_IN_STRIP,
	double TIME_BETWEEN_BLINKS_SECOND,
	double NUMBER_OF_LEDS_PER_METER,
	double SCROLL_SPEED_PER_METER_PER_SECOND,
	Color DEFAULT_COLOR,
	int SATURATION,
	int VALUE
) {}
