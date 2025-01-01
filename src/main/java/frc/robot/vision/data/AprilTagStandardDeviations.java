package frc.robot.vision.data;

public record AprilTagStandardDeviations(
	double xAxisStandardDeviations,
	double yAxisStandardDeviations,
	double zAxisStandardDeviations,
	double rollStandardDeviations,
	double pitchStandardDeviations,
	double yawStandardDeviations
) {}
