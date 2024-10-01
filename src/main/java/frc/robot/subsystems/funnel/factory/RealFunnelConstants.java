package frc.robot.subsystems.funnel.factory;

import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
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
		SparkMaxWrapper sparkMAXWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.FUNNEL);

		SysIdRoutine.Config config = new SysIdRoutine.Config();

		sparkMAXWrapper.setSmartCurrentLimit(30);
		sparkMAXWrapper.getEncoder().setPositionConversionFactor(1);
		sparkMAXWrapper.getEncoder().setVelocityConversionFactor(1);

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMAXWrapper, config);

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMAXWrapper::getVoltage);

		Supplier<Double> position = () -> sparkMAXWrapper.getEncoder().getPosition();
		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", position, AngleUnit.ROTATIONS);

		BooleanSupplier isShooterBeamBroken = () -> sparkMAXWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMAXWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput shooterBeamBreaker = new SuppliedDigitalInput(isShooterBeamBroken, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		BooleanSupplier isAmpBeamBroken = () -> sparkMAXWrapper.getForwardLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMAXWrapper.getForwardLimitSwitch(FORWARD_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput ampBeamBreaker = new SuppliedDigitalInput(isAmpBeamBroken, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new FunnelStuff(logPath, motor, voltageSignal, positionSignal, shooterBeamBreaker, ampBeamBreaker);
	}

}
