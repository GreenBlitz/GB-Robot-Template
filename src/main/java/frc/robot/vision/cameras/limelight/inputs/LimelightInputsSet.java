package frc.robot.vision.cameras.limelight.inputs;


public record LimelightInputsSet(
	MTInputs mt1Inputs,
	MTInputs mt2Inputs,
	NeuralDetectionInputs neuralDetectionInputs,
	ColorDetectionInputs colorDetectionInputs,
	LimelightHardwareInputs hardwareInputs
) {

	public LimelightInputsSet() {
		this(
			new MTInputs(),
			new MTInputs(),
			new NeuralDetectionInputs(),
			new ColorDetectionInputs(),
			new LimelightHardwareInputs()
		);
	}

}
