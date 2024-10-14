package frc.robot.subsystems.lifter.factory;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.lifter.LifterStuff;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

import static edu.wpi.first.math.util.Units.inchesToMeters;

public class LifterRealConstants {

	private static final double DRUM_RADIUS = inchesToMeters(0.96);
	private static final int MOTOR_CONFIGURATION_TRIES = 5;
	private static final int DIGITAL_INPUT_ID = 9;
	private static final double DEBOUNCE_TIME = 0.05;

	private static TalonFXConfiguration generateMotorConfiguration() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.Feedback.SensorToMechanismRatio = 7 * (60.0 / 24.0);


		return configuration;
	}

	private static IDigitalInput generateLimitSwitch() {
		return new ChanneledDigitalInput(DIGITAL_INPUT_ID, DEBOUNCE_TIME, true);
	}

	//@formatter:off
	protected static LifterStuff generateLifterStuff(String logPath) {
		TalonFXWrapper talonFXWrapper = new TalonFXWrapper(IDs.TalonFXIDs.LIFTER);
		if (!talonFXWrapper.applyConfiguration(generateMotorConfiguration(), MOTOR_CONFIGURATION_TRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "lifter motor was not configured").report();
		}

		return new LifterStuff(
			logPath,
			new TalonFXMotor(logPath, talonFXWrapper, new SysIdRoutine.Config()),
			DRUM_RADIUS,
			generateLimitSwitch(),
			Phoenix6SignalBuilder.generatePhoenix6Signal(
					talonFXWrapper.getPosition(),
					GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
					AngleUnit.ROTATIONS
			)
		);
	}
	//@formatter:on
}

