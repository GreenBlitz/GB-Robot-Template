package frc.utils.ctre;


public record CTREDeviceID(int ID, BusChain busChain) {

	public CTREDeviceID(int ID) {
		this(ID, BusChain.ROBORIO);
	}

}
