package frc.robot.vision.cameras.photon.inputs;

public record PhotonInputsSet(PhotonAprilTagDetectionInputsAutoLogged aprilTagDetectionInputs, PhotonHardwareInputsAutoLogged hardwareInputs) {

	public PhotonInputsSet() {
		this(new PhotonAprilTagDetectionInputsAutoLogged(), new PhotonHardwareInputsAutoLogged());
	}

}
