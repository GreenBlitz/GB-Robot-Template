package frc.robot.subsystems.arm.factory;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.arm.Arm;

import java.util.function.Supplier;

public class ArmFactory {

	public static Arm create(String logPath, Supplier<Rotation2d> currentReverseSoftLimitSupplier) {
		return KrakenX60ArmBuilder.build(logPath, currentReverseSoftLimitSupplier);
	}

}
