package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.GlobalConstants;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.signal.phoenix.Phoenix6AngleSignal;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

public class FlywheelRealConstants {

	protected static final Rotation2d TOLERANCE = Rotation2d.fromRotations(0.5);

	protected static final TalonFXConfiguration CONFIGURATION = new TalonFXConfiguration();
	protected static final SysIdRoutine.Config SYSID_CONFIG = new SysIdRoutine.Config(
		Volts.of(1).per(Seconds.of(1)),
		Volts.of(7),
		Seconds.of(10),
		(state) -> SignalLogger.writeString("state", state.toString())
	);

	static {
		Slot0Configs PID_SLOT_0 = new Slot0Configs();
		PID_SLOT_0.kP = 0.05;
		CONFIGURATION.withSlot0(PID_SLOT_0);
	}

	public static TalonFXMotor getTalonFXMotor(String logPath, TalonFXWrapper motorWrapper) {
		return new TalonFXMotor(logPath, motorWrapper, SYSID_CONFIG);
	}

	public static Phoenix6AngleSignal generateSignal(StatusSignal<Double> statusSignal) {
		return Phoenix6SignalBuilder.generatePhoenix6Signal(statusSignal, GlobalConstants.ROBORIO_CANBUS_UPDATE_FREQUENCY, AngleUnit.ROTATIONS);
	}

}
