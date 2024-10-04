package frc.robot.subsystems.intake.factory;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeStuff;

import java.util.function.BooleanSupplier;

public class RealIntakeConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	private static void configMotor(SparkMaxWrapper motor) {
		motor.setInverted(false);
		motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
		motor.setSmartCurrentLimit(30);
		motor.getEncoder().setPositionConversionFactor(IntakeConstants.GEAR_RATIO);
		motor.getEncoder().setVelocityConversionFactor(IntakeConstants.GEAR_RATIO);
	}

	public static IntakeStuff generateIntakeStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMAXs.INTAKE);
		configMotor(sparkMaxWrapper);

		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, new SysIdRoutine.Config());

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		BooleanSupplier isPressed = () -> sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isPressed, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new IntakeStuff(logPath, motor, voltageSignal, beamBreaker);
	}

}
