package frc.robot.subsystems.elevatorRoller.factory;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.motor.talonsrx.TalonSRXMotor;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerConstants;
import frc.robot.subsystems.elevatorRoller.ElevatorRollerStuff;


public class RealElevatorRollerConstants {

	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;

	private final static int CURRENT_LIMIT = 30;

	private static void configMotor(TalonSRX motor) {
		motor.configContinuousCurrentLimit(CURRENT_LIMIT);
		motor.configPeakCurrentLimit(CURRENT_LIMIT);
		motor.enableCurrentLimit(true);
		motor.setNeutralMode(NeutralMode.Coast);
		motor.setInverted(true);
	}

	public static ElevatorRollerStuff generateRollerElevatorStuff(String logPath) {
		TalonSRX talonSRX = new TalonSRX(IDs.TalonSRXIDs.ELEVATOR_ROLLER);
		configMotor(talonSRX);

		TalonSRXMotor talonSRXMotor = new TalonSRXMotor(logPath, talonSRX, ElevatorRollerConstants.GEAR_RATIO);

		talonSRX.overrideLimitSwitchesEnable(false);
		ChanneledDigitalInput beamBreaker = new ChanneledDigitalInput(0, DEBOUNCE_TIME_SECONDS, true);

		SuppliedDoubleSignal voltage = new SuppliedDoubleSignal("voltage", talonSRX::getMotorOutputVoltage);

		return new ElevatorRollerStuff(logPath, talonSRXMotor, beamBreaker, voltage);
	}

}
