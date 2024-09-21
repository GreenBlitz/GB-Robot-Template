package frc.robot.hardware.phoenix6;

public record Phoenix6DeviceID(int ID, BusChain busChain) {

	public Phoenix6DeviceID(int ID) {
		this(ID, BusChain.ROBORIO);
	}

}
