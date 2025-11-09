package frc.robot.vision.cameras.limelight.inputs;

public record LimelightInputs(
	MtPoseObservationInputsAutoLogged mt1PoseObservationInputs,
	MtPoseObservationInputsAutoLogged mt2PoseObservationInputs,
	DetectedObjectObservationsInputsAutoLogged detectedObjectObservationsInputs
) {}
