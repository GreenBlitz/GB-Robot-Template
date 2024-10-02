package frc.robot.subsystems.elevatorRoller;

import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.motor.IMotor;

public record ElevatorRollerAvatiach(String logPath, String digitalInputLogPath, IMotor motor, IDigitalInput digitalInput) {

	public ElevatorRollerAvatiach(String logPath, IMotor motor, IDigitalInput digitalInput) {
		this(logPath, logPath + "digitalInput", motor, digitalInput);
	}

}
