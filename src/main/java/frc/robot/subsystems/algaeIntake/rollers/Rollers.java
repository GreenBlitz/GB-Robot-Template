package frc.robot.subsystems.algaeIntake.rollers;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.DistanceUnit;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;

public class Rollers extends GBSubsystem {

	private final ControllableMotor rollers;
	private final InputSignal<Double> voltageSignal;
	private final CANrange canRange;
	private final RollersCommandsBuilder commandsBuilder;

	public Rollers(String logPath, ControllableMotor rollers, InputSignal<Double> voltageSignal, CANrange canRange){
		super(logPath);
		this.rollers = rollers;
		this.voltageSignal = voltageSignal;
		this.canRange = canRange;
		this.commandsBuilder = new RollersCommandsBuilder(this);
		setDefaultCommand(commandsBuilder.stop());

		periodic();
	}

	public RollersCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public double getVoltage() {
		return voltageSignal.getLatestValue();
	}

	public boolean isAlgaeIn(){
		return canRange.getDistance().getValueAsDouble() < RollersConstants.DISTANCE_FROM_CAN_RANGE_TO_DETECT_ALGAE_METERS;
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
	}

	private void updateInputs() {
		rollers.updateSimulation();
		rollers.updateInputs(voltageSignal);
		canRange.getDistance().refresh();
	}

	public void setBrake(boolean brake) {
		rollers.setBrake(brake);
	}

	protected void setPower(double power) {
		rollers.setPower(power);
	}

	protected void stop() {
		setPower(0);
	}

}
