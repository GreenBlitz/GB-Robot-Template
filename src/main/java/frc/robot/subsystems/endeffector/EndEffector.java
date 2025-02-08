package frc.robot.subsystems.endeffector;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.joysticks.SmartJoystick;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.interfaces.ControllableMotor;
import frc.robot.hardware.interfaces.InputSignal;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

public class EndEffector extends GBSubsystem {

	private final ControllableMotor roller;
	private final InputSignal<Double> powerSignal;
	private final InputSignal<Double> currentSignal;
	// Todo: now there is only one bb connected. We need to add one in the future.
//	private final IDigitalInput beamBreakerNotWorking;
//	private final DigitalInputInputsAutoLogged beamBreakerNotWorkingInputs;
	private final IDigitalInput backBeamBreaker;
	private final DigitalInputInputsAutoLogged backBeamBreakerInputs;
	private final EndEffectorCommandsBuilder commandsBuilder;
	private double calibrationPower = 0;

	public EndEffector(
		String logPath,
		ControllableMotor roller,
		InputSignal<Double> powerSignal,
		InputSignal<Double> currentSignal,
		IDigitalInput beamBreakerNotWorking,
		IDigitalInput backBeamBreaker
	) {
		super(logPath);
		this.roller = roller;
		this.powerSignal = powerSignal;
		this.currentSignal = currentSignal;

//		this.beamBreakerNotWorking = beamBreakerNotWorking;
//		this.beamBreakerNotWorkingInputs = new DigitalInputInputsAutoLogged();

		this.backBeamBreaker = backBeamBreaker;
		this.backBeamBreakerInputs = new DigitalInputInputsAutoLogged();

		this.commandsBuilder = new EndEffectorCommandsBuilder(this);

		periodic();

		setDefaultCommand(commandsBuilder.setPower(0));
	}

	public EndEffectorCommandsBuilder getCommandsBuilder() {
		return commandsBuilder;
	}

	public boolean isCoralInFront() {
		return backBeamBreakerInputs.debouncedValue; // beamBreakerNotWorkingInputs.debouncedValue;
	}

	public boolean isCoralInBack() {
		return backBeamBreakerInputs.debouncedValue;
	}

	public double getPower() {
		return powerSignal.getLatestValue();
	}

	public double getCurrent() {
		return currentSignal.getLatestValue();
	}

	@Override
	protected void subsystemPeriodic() {
		updateInputs();
		log();
	}

	private void updateInputs() {
		roller.updateSimulation();
		roller.updateInputs(powerSignal, currentSignal);

//		beamBreakerNotWorking.updateInputs(beamBreakerNotWorkingInputs);
//		Logger.processInputs(getLogPath() + "/BeamBreakerNotWorking", beamBreakerNotWorkingInputs);

		backBeamBreaker.updateInputs(backBeamBreakerInputs);
		Logger.processInputs(getLogPath() + "/BackBeamBreaker", backBeamBreakerInputs);
	}

	private void log() {
		Logger.recordOutput(getLogPath() + "/isCoralInFront", isCoralInFront());
		Logger.recordOutput(getLogPath() + "/isCoralInBack", isCoralInBack());
	}

	public void setBrake(boolean brake) {
		roller.setBrake(brake);
	}

	protected void stop() {
		roller.stop();
	}

	protected void setPower(double power) {
		roller.setPower(power);
	}

	public void applyCalibrationsBindings(SmartJoystick joystick) {
		joystick.R1.whileTrue(commandsBuilder.setPower(() -> calibrationPower));

		joystick.B.onTrue(new InstantCommand(() -> calibrationPower = Math.max(calibrationPower - 0.01, -1)));
		joystick.X.onTrue(new InstantCommand(() -> calibrationPower = Math.min(calibrationPower + 0.01, 1)));
		joystick.A.onTrue(new InstantCommand(() -> calibrationPower = Math.max(calibrationPower - 0.1, -1)));
		joystick.Y.onTrue(new InstantCommand(() -> calibrationPower = Math.min(calibrationPower + 0.1, 1)));

		joystick.POV_LEFT.onTrue(commandsBuilder.setPower(EndEffectorState.KEEP.getPower()));
		joystick.POV_DOWN.onTrue(commandsBuilder.setPower(EndEffectorState.INTAKE.getPower()).until(this::isCoralInBack));
		joystick.POV_UP.onTrue(commandsBuilder.setPower(EndEffectorState.OUTTAKE.getPower()).until(() -> !isCoralInFront()));
		joystick.POV_RIGHT.onTrue(commandsBuilder.stop());
	}

}
