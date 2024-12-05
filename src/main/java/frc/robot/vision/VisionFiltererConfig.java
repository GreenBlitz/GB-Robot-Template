package frc.robot.vision;

import frc.robot.vision.rawdata.RawVisionData;
import java.util.function.BiFunction;

public record VisionFiltererConfig(
	String logPath,
	VisionFiltersTolerances VisionFiltersTolerances,
	BiFunction<RawVisionData, VisionFiltersTolerances, Boolean> filters
) {}
