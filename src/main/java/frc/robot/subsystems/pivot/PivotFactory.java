package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.controls.PositionVoltage;
import frc.robot.RobotType;
import frc.robot.constants.IDs;
import frc.robot.hardware.motor.phoenix6.TalonFXMotor;
import frc.robot.hardware.motor.phoenix6.TalonFXWrapper;
import frc.robot.hardware.request.phoenix6.Phoenix6AngleRequest;
import frc.robot.hardware.signal.phoenix.Phoenix6SignalBuilder;
import frc.utils.AngleUnit;

public class PivotFactory {

	public static Pivot generatePivot(RobotType robotType, String logPath) {
		return switch (robotType) {
			case REAL -> getRealPivot(logPath);
			default -> null;
		};
	}

	private static Pivot getRealPivot(String logPath) {
		TalonFXWrapper motorWrapper = new TalonFXWrapper(IDs.TalonFXs.PIVOT_MOTOR_ID);
		motorWrapper.applyConfiguration(PivotRealConstants.TALON_FX_CONFIGURATION);

		return new Pivot(
			logPath,
			PivotRealConstants.getTalonFXMotor(logPath, motorWrapper),
			new Phoenix6AngleRequest(new PositionVoltage(0).withEnableFOC(true)),
			Phoenix6SignalBuilder.generatePhoenix6Signal(motorWrapper.getPosition(), 100, AngleUnit.ROTATIONS)
		);
	}

}
