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
import frc.robot.constants.MathConstants;
import frc.utils.GBSubsystem;
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
    private double lastGyroYawRads = 0.0;
    private double accumGyroYawRads = 0.0;
    private double wheelRadiusMeters = 0.0;

    public WheelRadiusCharacterization(
            GBSubsystem drive, double driveRadiusMeters, Rotation2d characterizationSpeed,
            Supplier<Rotation2d[]> wheelDriveDistanceSupplier, Supplier<Rotation2d> gyroAngleSupplier,
            Consumer<Rotation2d> velocityControl, Runnable onEnd
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
        double currentGyroYawRads = gyroAngleSupplier.get().getRadians();
        accumGyroYawRads += MathUtil.angleModulus(currentGyroYawRads - lastGyroYawRads);
        lastGyroYawRads = currentGyroYawRads;
    }

    private double getDriveDistanceMeters() {
        return accumGyroYawRads * driveRadiusMeters;
    }

    private double getAverageDriveDistanceRads() {
        Rotation2d[] wheelPositions = wheelDriveDistanceSupplier.get();
        double averageDriveDistanceRads = 0.0;
        for (int i = 0; i < WheelRadiusConstants.NUMBER_OF_MODULES; i++) {
            averageDriveDistanceRads += Math.abs(wheelPositions[i].getRadians() - startWheelPositions[i].getRadians());
        }
        return averageDriveDistanceRads / WheelRadiusConstants.NUMBER_OF_MODULES;
    }

    @Override
    public void initialize() {
        lastGyroYawRads = gyroAngleSupplier.get().getRadians();
        accumGyroYawRads = 0.0;

        startWheelPositions = wheelDriveDistanceSupplier.get();
    }

    @Override
    public void execute() {
        velocityControl.accept(characterizationSpeed);

        updateAnglePassedFromStart();

        // Distance meters / Distance Radians = Radius Meters
        wheelRadiusMeters = getDriveDistanceMeters() / getAverageDriveDistanceRads();
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.run();
        String output = accumGyroYawRads <= MathConstants.FULL_CIRCLE.getRadians()
                ? "Not enough data for characterization"
                : wheelRadiusMeters + " meters";
        Logger.recordOutput(WheelRadiusConstants.LOG_PATH, output);
    }

}
