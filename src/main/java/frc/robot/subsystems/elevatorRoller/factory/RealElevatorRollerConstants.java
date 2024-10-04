package frc.robot.subsystems.elevatorRoller.factory;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushedSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerStuff;
import com.revrobotics.SparkLimitSwitch;

import java.util.function.BooleanSupplier;

public class RealElevatorRollerConstants {

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;
	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;
	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private static void configMotor(SparkMaxWrapper motor) {
		motor.getEncoder().setPositionConversionFactor(ElevatorRollerConstants.GEAR_RATIO);
		motor.getEncoder().setVelocityConversionFactor(ElevatorRollerConstants.GEAR_RATIO);
		motor.setSmartCurrentLimit(40);
		motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
	}

	public static ElevatorRollerStuff generateRollerStuff(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.SparkMaxIDs.ELEVATOR_ROLLER);
		configMotor(sparkMaxWrapper);
		BrushedSparkMAXMotor brushedSparkMAXMotor = new BrushedSparkMAXMotor(logPath, sparkMaxWrapper);

		BooleanSupplier isBeamBroken = () -> sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isBeamBroken, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		SuppliedDoubleSignal voltage = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);

		return new ElevatorRollerStuff(logPath, brushedSparkMAXMotor, beamBreaker, voltage);
	}

}
