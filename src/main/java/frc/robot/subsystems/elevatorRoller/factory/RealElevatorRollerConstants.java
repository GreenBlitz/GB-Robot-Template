package frc.robot.subsystems.elevatorRoller.factory;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushedSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerAvatiach;
import com.revrobotics.SparkLimitSwitch;

import java.util.function.BooleanSupplier;

public class RealElevatorRollerConstants {

	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;
	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;
	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	public static ElevatorRollerAvatiach generate(String logPath) {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.ELEVATOR_ROLLER);

		BrushedSparkMAXMotor brushedSparkMAXMotor = new BrushedSparkMAXMotor(logPath, sparkMaxWrapper);

		BooleanSupplier isBeamBroken = () -> sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMaxWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput beamBreaker = new SuppliedDigitalInput(isBeamBroken, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new ElevatorRollerAvatiach(logPath, brushedSparkMAXMotor, beamBreaker);
	}

}
