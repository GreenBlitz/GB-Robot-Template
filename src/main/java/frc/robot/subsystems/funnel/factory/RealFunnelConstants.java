package frc.robot.subsystems.funnel.factory;

import com.revrobotics.CANSparkBase;
import com.revrobotics.SparkLimitSwitch;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.funnel.FunnelConstants;
import frc.robot.subsystems.funnel.FunnelStuff;

import java.util.function.BooleanSupplier;

public class RealFunnelConstants {

	private final static double DEBOUNCE_TIME_SECONDS = 0.05;
	private final static Debouncer.DebounceType DEBOUNCE_TYPE = Debouncer.DebounceType.kBoth;
	private final static SparkLimitSwitch.Type REVERSE_LIMIT_SWITCH_TYPE = SparkLimitSwitch.Type.kNormallyOpen;

	protected static void configMotor(SparkMaxWrapper motor){
		motor.setIdleMode(CANSparkBase.IdleMode.kCoast);
		motor.setSmartCurrentLimit(30);
		motor.getEncoder().setPositionConversionFactor(FunnelConstants.GEAR_RATIO);
		motor.getEncoder().setVelocityConversionFactor(FunnelConstants.GEAR_RATIO);
	}

	public static FunnelStuff generateFunnelStuff(String logPath) {
		SparkMaxWrapper sparkMAXWrapper = new SparkMaxWrapper(IDs.CANSparkMAXIDs.FUNNEL);
		SysIdRoutine.Config config = new SysIdRoutine.Config();
		configMotor(sparkMAXWrapper);
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(logPath, sparkMAXWrapper, config);

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMAXWrapper::getVoltage);

		BooleanSupplier isShooterBeamBroken = () -> sparkMAXWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).isPressed();
		sparkMAXWrapper.getReverseLimitSwitch(REVERSE_LIMIT_SWITCH_TYPE).enableLimitSwitch(false);
		SuppliedDigitalInput shooterBeamBreaker = new SuppliedDigitalInput(isShooterBeamBroken, DEBOUNCE_TYPE, DEBOUNCE_TIME_SECONDS);

		return new FunnelStuff(logPath, motor, voltageSignal, shooterBeamBreaker);
	}

}
