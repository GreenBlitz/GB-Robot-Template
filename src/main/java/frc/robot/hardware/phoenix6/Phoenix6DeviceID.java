package frc.robot.hardware.phoenix6;

import frc.robot.Robot;

import java.util.Objects;

public record Phoenix6DeviceID(int ID, BusChain busChain) {

	public Phoenix6DeviceID(int ID) {
		this(ID, BusChain.ROBORIO);
	}

	public Phoenix6DeviceID(int ID, BusChain busChain) {
		this.ID = ID;
		this.busChain = Objects.equals(determineBusChain(busChain), BusChain.CANIVORE.getChainName()) ? BusChain.CANIVORE : BusChain.ROBORIO;
	}

	public static String determineBusChain(BusChain busChain) {
		return Robot.ROBOT_TYPE.isSimulation() ? BusChain.ROBORIO.getChainName() : busChain.getChainName();
	}

}
