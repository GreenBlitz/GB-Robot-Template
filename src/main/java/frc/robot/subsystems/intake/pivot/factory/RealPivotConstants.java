package frc.robot.subsystems.intake.pivot.factory;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.sparkmax.BrushlessSparkMAXMotor;
import frc.robot.hardware.motor.sparkmax.SparkMaxWrapper;
import frc.robot.hardware.signal.supplied.SuppliedAngleSignal;
import frc.robot.hardware.signal.supplied.SuppliedDoubleSignal;
import frc.robot.subsystems.intake.pivot.PivotConstants;
import frc.robot.subsystems.intake.pivot.PivotStuff;
import frc.utils.AngleUnit;

public class RealPivotConstants {

	private static final double GEAR_RATIO = 1;

	public static PivotStuff generatePivotStuff() {
		SparkMaxWrapper sparkMaxWrapper = new SparkMaxWrapper(IDs.PIVOT_MOTOR_ID);
		sparkMaxWrapper.getAbsoluteEncoder().setPositionConversionFactor(GEAR_RATIO);
		sparkMaxWrapper.getAbsoluteEncoder().setVelocityConversionFactor(GEAR_RATIO);
		SysIdRoutine.Config config = new SysIdRoutine.Config();
		BrushlessSparkMAXMotor motor = new BrushlessSparkMAXMotor(PivotConstants.LOG_PATH, sparkMaxWrapper, config);

		SuppliedDoubleSignal voltageSignal = new SuppliedDoubleSignal("voltage", sparkMaxWrapper::getVoltage);
		SuppliedAngleSignal positionSignal = new SuppliedAngleSignal(
			"position",
			() -> sparkMaxWrapper.getAbsoluteEncoder().getPosition(),
			AngleUnit.ROTATIONS
		);

		return new PivotStuff(PivotConstants.LOG_PATH, motor, voltageSignal, positionSignal, sparkMaxWrapper.getAbsoluteEncoder());
	}

}
