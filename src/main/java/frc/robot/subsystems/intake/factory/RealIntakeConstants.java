package frc.robot.subsystems.intake.factory;

import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.cansparkmax.SparkMaxDoubleSignal;
import frc.robot.subsystems.intake.IntakeStuff;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealIntakeConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	public static IntakeStuff generateIntakeStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMaxs.INTAKE);
		SysIdRoutine.Config config = new SysIdRoutine.Config();
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, config);

		Supplier<Double> voltage = () -> (sparkMaxWrapper.getBusVoltage() * sparkMaxWrapper.getAppliedOutput());
		SparkMaxDoubleSignal signal = new SparkMaxDoubleSignal("voltage", voltage);

		BooleanSupplier isPressed = () -> sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);

		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isPressed, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new IntakeStuff(logPath, motor, signal, beamBreaker);
	}

}
