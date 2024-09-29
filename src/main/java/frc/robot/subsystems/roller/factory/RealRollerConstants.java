package frc.robot.subsystems.roller.factory;

import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.cansparkmax.SparkMaxAngleSignal;
import frc.robot.hardware.signal.cansparkmax.SparkMaxDoubleSignal;
import frc.robot.subsystems.roller.RollerStuff;
import frc.utils.AngleUnit;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealRollerConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	private final static double GEAR_RATIO = 1.0 / 6.0;

	public static RollerStuff generateIntakeStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMAXs.ROLLER);
		sparkMaxWrapper.getEncoder().setPositionConversionFactor(GEAR_RATIO);
		sparkMaxWrapper.getEncoder().setVelocityConversionFactor(GEAR_RATIO);
		SysIdRoutine.Config config = new SysIdRoutine.Config();
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, config);

		Supplier<Double> voltage = () -> (sparkMaxWrapper.getBusVoltage() * sparkMaxWrapper.getAppliedOutput());
		SparkMaxDoubleSignal voltageSignal = new SparkMaxDoubleSignal("voltage", voltage);

		Supplier<Double> position = () -> sparkMaxWrapper.getEncoder().getPosition();
		SparkMaxAngleSignal angleSignal = new SparkMaxAngleSignal("position", position, AngleUnit.ROTATIONS);

		BooleanSupplier isBeamBroke = () -> sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);

		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isBeamBroke, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new RollerStuff(logPath, motor, voltageSignal, angleSignal, beamBreaker);
	}

}
