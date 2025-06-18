package frc.robot.newvision.cameras.limelight;

import frc.utils.alerts.Alert;

public class LimelightTarget2dValues {

	private final int AMOUNT_OF_TARGET2D_VALUES = 17;

	private boolean isValid;
	private int targetCount;
	private double targetLatencyMilliseconds;
	private double captureLatencyMilliseconds;
	private double targetXDegrees;
	private double targetYDegrees;
	private double targetXNoCrosshairDegrees;
	private double targetYNoCrosshairDegrees;
	private double targetAreaPercentage;
	private int targetID;
	private int targetDetectorClassIndex;
	private int targetClassifierClassIndex;
	private double targetLongSidePixels;
	private double targetShortSidePixels;
	private double targetHorizontalExtentPixels;
	private double targetVerticalExtentPixels;
	private double targetSkewDegrees;

	public LimelightTarget2dValues() {
		setValues(false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	}

	public void setValues(
		boolean isValid,
		int targetCount,
		double targetLatencyMilliseconds,
		double captureLatencyMilliseconds,
		double targetXDegrees,
		double targetYDegrees,
		double targetXNoCrosshairDegrees,
		double targetYNoCrosshairDegrees,
		double targetAreaPercentage,
		int targetID,
		int targetDetectorClassIndex,
		int targetClassifierClassIndex,
		double targetLongSidePixels,
		double targetShortSidePixels,
		double targetHorizontalExtentPixels,
		double targetVerticalExtentPixels,
		double targetSkewDegrees
	) {
		this.isValid = isValid;
		this.targetCount = targetCount;
		this.targetLatencyMilliseconds = targetLatencyMilliseconds;
		this.captureLatencyMilliseconds = captureLatencyMilliseconds;
		this.targetXDegrees = targetXDegrees;
		this.targetYDegrees = targetYDegrees;
		this.targetXNoCrosshairDegrees = targetXNoCrosshairDegrees;
		this.targetYNoCrosshairDegrees = targetYNoCrosshairDegrees;
		this.targetAreaPercentage = targetAreaPercentage;
		this.targetID = targetID;
		this.targetDetectorClassIndex = targetDetectorClassIndex;
		this.targetClassifierClassIndex = targetClassifierClassIndex;
		this.targetLongSidePixels = targetLongSidePixels;
		this.targetShortSidePixels = targetShortSidePixels;
		this.targetHorizontalExtentPixels = targetHorizontalExtentPixels;
		this.targetVerticalExtentPixels = targetVerticalExtentPixels;
		this.targetSkewDegrees = targetSkewDegrees;
	}

	public void setValues(double[] target2dArray, String logPath) {
		if (target2dArray.length != AMOUNT_OF_TARGET2D_VALUES) {
			new Alert(Alert.AlertType.WARNING, logPath + "/invalid target2d array received").report();
			setValues(false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		} else {
			setValues(
				target2dArray[0] == 1,
				(int) target2dArray[1],
				target2dArray[2],
				target2dArray[3],
				target2dArray[4],
				target2dArray[5],
				target2dArray[6],
				target2dArray[7],
				target2dArray[8],
				(int) target2dArray[9],
				(int) target2dArray[10],
				(int) target2dArray[11],
				target2dArray[12],
				target2dArray[13],
				target2dArray[14],
				target2dArray[15],
				target2dArray[16]
			);
		}
	}

	public boolean isValid() {
		return isValid;
	}

	public int getTargetCount() {
		return targetCount;
	}

	public double getTargetLatencyMilliseconds() {
		return targetLatencyMilliseconds;
	}

	public double getCaptureLatencyMilliseconds() {
		return captureLatencyMilliseconds;
	}

	public double getTargetXDegrees() {
		return targetXDegrees;
	}

	public double getTargetYDegrees() {
		return targetYDegrees;
	}

	public double getTargetXNoCrosshairDegrees() {
		return targetXNoCrosshairDegrees;
	}

	public double getTargetYNoCrosshairDegrees() {
		return targetYNoCrosshairDegrees;
	}

	public double getTargetAreaPercentage() {
		return targetAreaPercentage;
	}

	public int getTargetID() {
		return targetID;
	}

	public int getTargetDetectorClassIndex() {
		return targetDetectorClassIndex;
	}

	public int getTargetClassifierClassIndex() {
		return targetClassifierClassIndex;
	}

	public double getTargetLongSidePixels() {
		return targetLongSidePixels;
	}

	public double getTargetShortSidePixels() {
		return targetShortSidePixels;
	}

	public double getTargetHorizontalExtentPixels() {
		return targetHorizontalExtentPixels;
	}

	public double getTargetVerticalExtentPixels() {
		return targetVerticalExtentPixels;
	}

	public double getTargetSkewDegrees() {
		return targetSkewDegrees;
	}

}
