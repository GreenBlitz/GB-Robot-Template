// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils.calibration.swervecalibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.constants.MathConstants;
import frc.robot.subsystems.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class WheelRadiusCharacterization extends Command {

	private final Supplier<Rotation2d> gyroAngleSupplier;
	private final Supplier<Rotation2d[]> wheelDriveDistanceSupplier;
	private final Consumer<Rotation2d> velocityControl;
	private final Rotation2d characterizationSpeed;
	private final Runnable onEnd;
	private final double driveRadiusMeters;

	private Rotation2d[] startWheelPositions;
	private double lastGyroYawRadians = 0.0;
	private double accumulateGyroYawRadians = 0.0;
	private double wheelRadiusMeters = 0.0;

	public WheelRadiusCharacterization(
		GBSubsystem drive,
		double driveRadiusMeters,
		Rotation2d characterizationSpeed,
		Supplier<Rotation2d[]> wheelDriveDistanceSupplier,
		Supplier<Rotation2d> gyroAngleSupplier,
		Consumer<Rotation2d> velocityControl,
		Runnable onEnd
	) {
		this.characterizationSpeed = characterizationSpeed;
		this.driveRadiusMeters = driveRadiusMeters;
		this.wheelDriveDistanceSupplier = wheelDriveDistanceSupplier;
		this.gyroAngleSupplier = gyroAngleSupplier;
		this.velocityControl = velocityControl;
		this.onEnd = onEnd;
		addRequirements(drive);
	}

	private void updateAnglePassedFromStart() {
		double currentGyroYawRadians = gyroAngleSupplier.get().getRadians();
		accumulateGyroYawRadians += MathUtil.angleModulus(currentGyroYawRadians - lastGyroYawRadians);
		lastGyroYawRadians = currentGyroYawRadians;
	}

	private double getDriveDistanceMeters() {
		return accumulateGyroYawRadians * driveRadiusMeters;
	}

	private double getAverageDriveDistanceRadians() {
		Rotation2d[] wheelPositions = wheelDriveDistanceSupplier.get();
		double averageDriveDistanceRadians = 0.0;
		for (int i = 0; i < WheelRadiusConstants.NUMBER_OF_MODULES; i++) {
			averageDriveDistanceRadians += Math.abs(wheelPositions[i].getRadians() - startWheelPositions[i].getRadians());
		}
		return averageDriveDistanceRadians / WheelRadiusConstants.NUMBER_OF_MODULES;
	}

	@Override
	public void initialize() {
		lastGyroYawRadians = gyroAngleSupplier.get().getRadians();
		accumulateGyroYawRadians = 0.0;

		startWheelPositions = wheelDriveDistanceSupplier.get();
	}

	@Override
	public void execute() {
		velocityControl.accept(characterizationSpeed);

		updateAnglePassedFromStart();

		// Distance meters / Distance Radians = Radius Meters
		wheelRadiusMeters = getDriveDistanceMeters() / getAverageDriveDistanceRadians();
	}

	@Override
	public void end(boolean interrupted) {
		onEnd.run();
		String output = accumulateGyroYawRadians <= MathConstants.FULL_CIRCLE.getRadians()
			? "Not enough data for characterization"
			: wheelRadiusMeters + " meters";
		Logger.recordOutput(WheelRadiusConstants.LOG_PATH, output);
	}

}
