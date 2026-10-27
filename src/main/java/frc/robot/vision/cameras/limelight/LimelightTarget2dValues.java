package frc.robot.vision.cameras.limelight;

import org.wpilib.math.geometry.Angle2d;

public record LimelightTarget2dValues(
	boolean isValid,
	int targetCount,
	double targetLatencyMilliseconds,
	double captureLatencyMilliseconds,
	Angle2d targetX,
	Angle2d targetY,
	Angle2d targetXNoCrosshair,
	Angle2d targetYNoCrosshair,
	double targetAreaPercentage,
	int targetID,
	int targetDetectorClassIndex,
	int targetClassifierClassIndex,
	double targetLongSidePixels,
	double targetShortSidePixels,
	double targetHorizontalExtentPixels,
	double targetVerticalExtentPixels,
	Angle2d targetSkew
) {

	public static final int AMOUNT_OF_TARGET2D_VALUES = 17;

	public LimelightTarget2dValues() {
		this(false, 0, 0, 0, Angle2d.ZERO, Angle2d.ZERO, Angle2d.ZERO, Angle2d.ZERO, 0, 0, 0, 0, 0, 0, 0, 0, Angle2d.ZERO);
	}

	public static LimelightTarget2dValues fromArray(double[] target2dArray) {
		if (target2dArray.length == AMOUNT_OF_TARGET2D_VALUES) {
			return new LimelightTarget2dValues(
				target2dArray[0] == 1,
				(int) target2dArray[1],
				target2dArray[2],
				target2dArray[3],
				Angle2d.fromDegrees(target2dArray[4]),
				Angle2d.fromDegrees(target2dArray[5]),
				Angle2d.fromDegrees(target2dArray[6]),
				Angle2d.fromDegrees(target2dArray[7]),
				target2dArray[8],
				(int) target2dArray[9],
				(int) target2dArray[10],
				(int) target2dArray[11],
				target2dArray[12],
				target2dArray[13],
				target2dArray[14],
				target2dArray[15],
				Angle2d.fromDegrees(target2dArray[16])
			);
		} else {
			return new LimelightTarget2dValues();
		}
	}

}
