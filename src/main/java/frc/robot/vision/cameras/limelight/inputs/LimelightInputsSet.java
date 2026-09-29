package frc.robot.vision.cameras.limelight.inputs;


public record LimelightInputsSet(
	MTInputsAutoLogged mt1Inputs,
	MTInputsAutoLogged mt2Inputs,
	NeuralDetectionInputsAutoLogged neuralDetectionInputs,
	ColorDetectionInputsAutoLogged colorDetectionInputs,
	LimelightHardwareInputsAutoLogged hardwareInputs
) {

	public LimelightInputsSet() {
		this(
			new MTInputsAutoLogged(),
			new MTInputsAutoLogged(),
			new NeuralDetectionInputsAutoLogged(),
			new ColorDetectionInputsAutoLogged(),
			new LimelightHardwareInputsAutoLogged()
		);
	}

}
