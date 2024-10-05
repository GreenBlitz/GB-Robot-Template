package frc.robot.subsystems.intake.factory;

import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.intake.IntakeRollerStuff;

import java.util.function.BooleanSupplier;

public class IntakeRollerRealConstant {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	private final static int CONVERSION_FACTOR = 3;

	private final static int CURRENT_LIMIT = 30;

	public static IntakeRollerStuff generateIntakeRollerStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.INTAKE_ROLLER);
		SysIdRoutine.Config config = new SysIdRoutine.Config();
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMaxWrapper, config);
		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);
		sparkMaxWrapper.getEncoder().setVelocityConversionFactor(CONVERSION_FACTOR);
		sparkMaxWrapper.getEncoder().setPositionConversionFactor(CONVERSION_FACTOR);
		sparkMaxWrapper.setSmartCurrentLimit(CURRENT_LIMIT);

		BooleanSupplier isBeamBroken = () -> sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isBeamBroken, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new IntakeRollerStuff(logPath, motor, voltageSignal, beamBreaker);
	}

}
