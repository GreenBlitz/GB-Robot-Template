package frc.robot.led;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.GBSubsystem;

public class LEDStateHandler extends GBSubsystem {

	private final CANdle candle;

	public LEDStateHandler(String logPath, CANdle candle) {
		super(logPath);
		this.candle = candle;

		setDefaultCommand(new ConditionalCommand(setState(LEDState.IDLE), setState(LEDState.DISABLE), DriverStation::isEnabled));
	}

	public Command setState(LEDState state) {
		Command setStateCommand = new RunCommand(() -> candle.animate(state.getAnimation()), this).ignoringDisable(true);

		if (state == LEDState.HAS_CORAL) {
			return setStateCommand.withTimeout(LEDConstants.TIME_FOR_HAS_CORAL_STATE);
		}

		return setStateCommand;
	}

}
