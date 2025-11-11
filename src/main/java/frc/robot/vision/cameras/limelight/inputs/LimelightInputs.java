package frc.robot.vision.cameras.limelight.inputs;

import frc.robot.vision.cameras.limelight.LimelightTarget2dValues;
import frc.utils.LimelightHelpers;

public record LimelightInputs(
	MtInputsAutoLogged mt1Inputs,
	MtInputsAutoLogged mt2Inputs,
	ObjectDetectionInputsAutoLogged ObjectDetectionInputs
) {

	public LimelightHelpers.PoseEstimate getMt1RawData() {
		return mt1Inputs.mtRawData;
	}

	public LimelightHelpers.PoseEstimate getMt2RawData() {
		return mt2Inputs.mtRawData;
	}

	public LimelightTarget2dValues getTarget2dValues() {
		return ObjectDetectionInputs.target2dValues;
	}

}
