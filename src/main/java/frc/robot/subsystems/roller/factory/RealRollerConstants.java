package frc.robot.subsystems.roller.factory;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerStuff;
import frc.utils.AngleUnit;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class RealRollerConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.1;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	private static void configMotor(SparkMaxWrapper motor) {
		motor.setInverted(false);
		motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
		motor.setSmartCurrentLimit(30);
		motor.getEncoder().setPositionConversionFactor(RollerConstants.GEAR_RATIO);
		motor.getEncoder().setVelocityConversionFactor(RollerConstants.GEAR_RATIO);
	}

	public static RollerStuff generateRollerStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMAXs.ROLLER);
		configMotor(sparkMaxWrapper);

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		Supplier<Double> positionSupplier = () -> sparkMaxWrapper.getEncoder().getPosition();
		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal("position", positionSupplier, AngleUnit.ROTATIONS);

		BooleanSupplier isBeamBroken = () -> sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isBeamBroken, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new RollerStuff(logPath, motor, voltageSignal, positionSignal, beamBreaker);
	}

}
