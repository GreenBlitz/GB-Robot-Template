package frc.robot.hardware.phoenix6;

import frc.robot.Robot;

public record Phoenix6DeviceID(int ID, BusChain busChain) {

	public Phoenix6DeviceID(int ID) {
		this(ID, BusChain.ROBORIO);
	}

	public static String determineBusChain(Phoenix6DeviceID ctreDeviceID) {
		return Robot.ROBOT_TYPE.isSimulation() ? BusChain.ROBORIO.getChainName() : ctreDeviceID.busChain().getChainName();
	}

}
