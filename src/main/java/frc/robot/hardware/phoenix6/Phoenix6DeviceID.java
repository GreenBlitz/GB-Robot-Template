package frc.robot.hardware.phoenix6;

import frc.robot.RobotOld;

public record Phoenix6DeviceID(int id, BusChain busChain) {

	public Phoenix6DeviceID(int id) {
		this(id, BusChain.ROBORIO);
	}

	public Phoenix6DeviceID(int id, BusChain busChain) {
		this.id = id;
		this.busChain = determineBusChain(busChain);
	}

	public static BusChain determineBusChain(BusChain busChain) {
		return RobotOld.ROBOT_TYPE.isSimulation() ? BusChain.ROBORIO : busChain;
	}

}
