package frc.robot.hardware.phoenix6;

import frc.robot.Robot;

public record Phoenix6DeviceID(int id, BusChain busChain) {

	public Phoenix6DeviceID {
		busChain = determineBusChain(busChain);
	}

	public Phoenix6DeviceID(int id) {
		this(id, BusChain.ROBORIO);
	}

	public static BusChain determineBusChain(BusChain busChain) {
		return Robot.ROBOT_TYPE.isSimulation() ? BusChain.ROBORIO : busChain;
	}

}
