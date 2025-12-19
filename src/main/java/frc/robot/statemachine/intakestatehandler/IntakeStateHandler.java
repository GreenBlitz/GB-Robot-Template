package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.roller.Roller;
import frc.utils.LoggedNetworkRotation2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class IntakeStateHandler {

	private final Arm fourBar;
	private final Roller rollers;
	private final IDigitalInput beamBreaker;
	private final String logPath;
	private final DigitalInputInputsAutoLogged beamBreakerInputs;
	private final LoggedNetworkNumber rollersCalibrationPower = new LoggedNetworkNumber("Tunable/IntakeRollerPower");
	private final LoggedNetworkRotation2d fourBarCalibrationPosition = new LoggedNetworkRotation2d("Tunable/FourBarPosition", new Rotation2d());

	private IntakeState currentState;

	public IntakeStateHandler(Arm fourBar, Roller rollers, IDigitalInput beamBreaker, String logPath) {
		this.fourBar = fourBar;
		this.rollers = rollers;
		this.beamBreaker = beamBreaker;
		this.beamBreakerInputs = new DigitalInputInputsAutoLogged();
		this.logPath = logPath + "/IntakeStateHandler";
		this.currentState = IntakeState.STAY_IN_PLACE;
	}

	public void periodic() {
		beamBreaker.updateInputs(beamBreakerInputs);
		Logger.processInputs(logPath, beamBreakerInputs);
	}

	public Command setState(IntakeState intakeState) {
		Command command;
		if (intakeState == IntakeState.CALIBRATION) {
			command = new ParallelCommandGroup(
				fourBar.getCommandsBuilder().setTargetPosition(fourBarCalibrationPosition::get),
				rollers.getCommandsBuilder().setPower(rollersCalibrationPower::get)
			);
		} else if (intakeState == IntakeState.STAY_IN_PLACE) {
			command = new ParallelCommandGroup(fourBar.getCommandsBuilder().stayInPlace(), rollers.getCommandsBuilder().stop());
		} else {
			command = new ParallelCommandGroup(
				fourBar.getCommandsBuilder().setTargetPosition(intakeState.getFourBarPosition()),
				rollers.getCommandsBuilder().setPower(intakeState.getIntakePower())
			);
		}

		return new ParallelCommandGroup(
			command,
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", intakeState.name())),
			new InstantCommand(() -> currentState = intakeState)
		);
	}

}
