package frc.robot.vision.cameras.limelight.inputs;

import frc.robot.hardware.ConnectedInputAutoLogged;

public record LimelightInputsSet(
	MTInputsAutoLogged mt1Inputs,
	MTInputsAutoLogged mt2Inputs,
	NeuralDetectionInputsAutoLogged neuralDetectionInputs,
	ColorDetectionInputsAutoLogged colorDetectionInputs,
	ConnectedInputAutoLogged connectedInput
) {

	public LimelightInputsSet() {
		this(
			new MTInputsAutoLogged(),
			new MTInputsAutoLogged(),
			new NeuralDetectionInputsAutoLogged(),
			new ColorDetectionInputsAutoLogged(),
			new ConnectedInputAutoLogged()
		);
	}

}
