package frc.robot.hardware.phoenix6;

import frc.robot.Robot;

public record Phoenix6DeviceID(int ID, BusChain busChain) {

	public Phoenix6DeviceID(int ID) {
		this(ID, BusChain.ROBORIO);
	}

	public Phoenix6DeviceID(int ID, BusChain busChain) {
		this.ID = ID;
		this.busChain = determineBusChain(busChain);
	}

	public static BusChain determineBusChain(BusChain busChain) {
		return Robot.ROBOT_TYPE.isSimulation() ? BusChain.ROBORIO : busChain;
	}

}
