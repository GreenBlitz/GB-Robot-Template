package frc.robot.vision.cameras.limelight.inputs;

public record LimelightInputsSet(
	MTInputsAutoLogged mt1Inputs,
	MTInputsAutoLogged mt2Inputs,
	NuralDetectionInputsAutoLogged nuralDetectionInputs,
	ColorDetectionInputsAutoLogged colorDetectionInputs
) {

	public LimelightInputsSet() {
		this(new MTInputsAutoLogged(), new MTInputsAutoLogged(), new NuralDetectionInputsAutoLogged(), new ColorDetectionInputsAutoLogged());
	}

}
