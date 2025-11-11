package frc.robot.vision.cameras.limelight.inputs;


public record LimelightInputs(
	MTInputsAutoLogged mt1Inputs,
	MTInputsAutoLogged mt2Inputs,
	ObjectDetectionInputsAutoLogged ObjectDetectionInputs
) {}
