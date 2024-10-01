package frc.robot.subsystems.funnel.factory;

import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.cansparkmax.SparkMaxAngleSignal;
import frc.robot.hardware.signal.cansparkmax.SparkMaxDoubleSignal;
import frc.robot.subsystems.funnel.FunnelStuff;
import frc.utils.AngleUnit;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealFunnelConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;
	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;
	private final static SparkLimitSwitch.Type FORWARD_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;


	public static FunnelStuff generateFunnelStuff(String logPath) {
		SparkMaxWrapper wrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.FUNNEL);
		SysIdRoutine.Config config = new SysIdRoutine.Config();
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, wrapper, config);

		Supplier<Double> voltage = () -> wrapper.getBusVoltage() * wrapper.getAppliedOutput();
		SparkMaxDoubleSignal voltageSignal = new SparkMaxDoubleSignal("voltage", voltage);

		Supplier<Double> position = () -> wrapper.getEncoder().getPosition();
		SparkMaxAngleSignal positionSignal = new SparkMaxAngleSignal("position", position, AngleUnit.ROTATIONS);

		BooleanSupplier isShooterBeamBroke = () -> wrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		wrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput shooterBeamBreaker = new SuppliedDigitalInput(isShooterBeamBroke, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		BooleanSupplier isAmpBeamBroke = () -> wrapper.getForwardLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		wrapper.getForwardLimitSwitch(FORWARD_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput ampBeamBreaker = new SuppliedDigitalInput(isAmpBeamBroke, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new FunnelStuff(logPath, motor, voltageSignal, positionSignal, shooterBeamBreaker, ampBeamBreaker);
	}

}
