// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils.calibration.swervecalibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.MathConstants;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class WheelRadiusCharacterization extends Command {

    private final double characterizationSpeed;

    private final double driveRadiusMeters;

    private final Supplier<Rotation2d> gyroAngleSupplier;

    private final Supplier<Rotation2d[]> wheelDriveDistanceSupplier;

    private final Consumer<Double> velocityControl;

    private final Runnable onEnd;

    private final CharacterizationDirection spinDirection;

    private Rotation2d[] startWheelPositions;

    private double lastGyroYawRads = 0.0;

    private double accumGyroYawRads = 0.0;

    private double wheelRadiusMeters = 0.0;

    public WheelRadiusCharacterization(
            GBSubsystem drive, CharacterizationDirection spinDirection, double driveRadiusMeters, double characterizationSpeed,
            Supplier<Rotation2d[]> wheelDriveDistanceSupplier, Supplier<Rotation2d> gyroAngleSupplier,
            Consumer<Double> velocityControl, Runnable onEnd
    ) {
        this.spinDirection = spinDirection;
        this.driveRadiusMeters = driveRadiusMeters;
        this.characterizationSpeed = characterizationSpeed;
        this.wheelDriveDistanceSupplier = wheelDriveDistanceSupplier;
        this.gyroAngleSupplier = gyroAngleSupplier;
        this.velocityControl = velocityControl;
        this.onEnd = onEnd;
        addRequirements(drive);
    }

    public double getAverageDriveDistanceRads() {
        Rotation2d[] wheelPositions = wheelDriveDistanceSupplier.get();
        double averageDriveDistanceRads = 0.0;
        for (int i = 0; i < WheelRadiusConstants.NUMBER_OF_MODULES; i++) {
            averageDriveDistanceRads += Math.abs(wheelPositions[i].minus(startWheelPositions[i]).getRadians());
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
        velocityControl.accept(spinDirection.getDirectionSign() * characterizationSpeed);

        // Update the angle passed from start
        double currentGyroYawRads = gyroAngleSupplier.get().getRadians();
        accumGyroYawRads += MathUtil.angleModulus(currentGyroYawRads - lastGyroYawRads);
        lastGyroYawRads = currentGyroYawRads;

        // Update the drive distance of the wheels
        double averageDriveDistanceRads = getAverageDriveDistanceRads();
        double driveDistanceMeters = accumGyroYawRads * driveRadiusMeters;

        // Distance meters / Distance Radians = Radius Meters
        wheelRadiusMeters = driveDistanceMeters / averageDriveDistanceRads;
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.run();
        if (accumGyroYawRads <= MathConstants.FULL_CIRCLE.getRadians()) {
            Logger.recordOutput(WheelRadiusConstants.LOG_PATH, "Not enough data for characterization");
        }
        else {
            Logger.recordOutput(
                    WheelRadiusConstants.LOG_PATH,
                    Units.metersToInches(wheelRadiusMeters) + " inches"
            );
        }
    }

}