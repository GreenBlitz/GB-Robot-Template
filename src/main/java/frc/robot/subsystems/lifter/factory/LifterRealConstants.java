package frc.robot.subsystems.lifter.factory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.robot.subsystems.lifter.LifterComponents;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;

import static edu.wpi.first.math.util.Units.inchesToMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class LifterRealConstants {

	private static final double DRUM_RADIUS_METERS = inchesToMeters(0.96);
	private static final int MOTOR_CONFIGURATION_TRIES = 5;

	private static TalonFXConfiguration generateMotorConfiguration() {
		TalonFXConfiguration configuration = new TalonFXConfiguration();

		configuration.Feedback.SensorToMechanismRatio = 7 * (60.0 / 24.0);

		return configuration;
	}

	private static SysIdRoutine.Config generateSysidConfig() {
		return new SysIdRoutine.Config(
			Volts.of(SysIdRoutineConfigConstants.RAMP_RATE_MAGNITUDE).per(Seconds.of(SysIdRoutineConfigConstants.RAMP_RATE_SECONDS)),
			Volts.of(SysIdRoutineConfigConstants.STEP_VOLTAGE),
			Seconds.of(SysIdRoutineConfigConstants.TIME_OUT),
			(state) -> SignalLogger.writeString("state", state.toString())
		);
	}

	//@formatter:off
	protected static LifterComponents generateLifterComponents(String logPath) {
		TalonFXWrapper talonFXWrapper = new TalonFXWrapper(IDs.TalonFXIDs.LIFTER);

		if (!talonFXWrapper.applyConfiguration(generateMotorConfiguration(), MOTOR_CONFIGURATION_TRIES).isOK()) {
			new Alert(Alert.AlertType.ERROR, logPath + "lifter motor was not configured").report();
		}

		return new LifterComponents(
			logPath,
			new TalonFXMotor(logPath, talonFXWrapper, generateSysidConfig()),
			DRUM_RADIUS_METERS,
			Phoenix6SignalBuilder.generatePhoenix6Signal(
					talonFXWrapper.getPosition(),
					GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
					AngleUnit.ROTATIONS
			)
		);
	}

    private static class SysIdRoutineConfigConstants {
        private static final double RAMP_RATE_MAGNITUDE = 1;
        private static final double RAMP_RATE_SECONDS = 1;
        private static final double STEP_VOLTAGE = 7;
        private static final double TIME_OUT = 10;
    }
	//@formatter:on
}

