package frc.robot.vision.cameras.limelight.inputs;

public record LimelightInputsSet(
	MTInputsAutoLogged mt1Inputs,
	MTInputsAutoLogged mt2Inputs,
	ObjectDetectionInputsAutoLogged ObjectDetectionInputs
) {

	public LimelightInputsSet() {
		this(new MTInputsAutoLogged(), new MTInputsAutoLogged(), new ObjectDetectionInputsAutoLogged());
	}

}
