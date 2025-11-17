package frc.robot.vision.cameras.limelight.inputs;

public record LimelightInputsSet(
	MT1InputsAutoLogged mt1Inputs,
	MT2InputsAutoLogged mt2Inputs,
	ObjectDetectionInputsAutoLogged ObjectDetectionInputs
) {

	public LimelightInputsSet() {
		this(new MT1InputsAutoLogged(), new MT2InputsAutoLogged(), new ObjectDetectionInputsAutoLogged());
	}

}
