// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils.calibration.swervecalibration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.utils.GBSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class WheelRadiusCharacterization extends Command {

    public enum Direction {
        CLOCKWISE(-1),
        COUNTER_CLOCKWISE(1);

        private final int directionSign;

        Direction(int directionSign) {
            this.directionSign = directionSign;
        }

        public int getDirectionSign() {
            return directionSign;
        }
    }

    private final Rotation2d characterizationSpeed;

    private final double driveRadiusMeters;

    private final GBSubsystem drive;

    private final Supplier<Rotation2d> gyroAngleSupplier;

    private final Supplier<Rotation2d[]> wheelDriveDistanceSupplier;

    private final Runnable onEnd;

    private final Consumer<Double> velocityControl;

    private final Direction omegaDirection;

    private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

    private double lastGyroYawRads = 0.0;

    private double accumGyroYawRads = 0.0;

    private Rotation2d[] startWheelPositions;

    private double currentEffectiveWheelRadius = 0.0;

    public WheelRadiusCharacterization(GBSubsystem drive, Direction omegaDirection, double driveRadiusMeters,
            Rotation2d characterizationSpeed,
            Supplier<Rotation2d[]> wheelDriveDistanceSupplier, Supplier<Rotation2d> gyroAngleSupplier,
            Consumer<Double> velocityControl, Runnable onEnd) {
        this.drive = drive;
        this.omegaDirection = omegaDirection;
        this.driveRadiusMeters = driveRadiusMeters;
        this.characterizationSpeed = characterizationSpeed;
        this.wheelDriveDistanceSupplier = wheelDriveDistanceSupplier;
        this.gyroAngleSupplier = gyroAngleSupplier;
        this.velocityControl = velocityControl;
        this.onEnd = onEnd;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        // Reset
        lastGyroYawRads = gyroAngleSupplier.get().getRadians();
        accumGyroYawRads = 0.0;

        startWheelPositions = wheelDriveDistanceSupplier.get();

        omegaLimiter.reset(0);
    }

    @Override
    public void execute() {
        // Run drive at velocity
        velocityControl.accept(omegaLimiter.calculate(omegaDirection.getDirectionSign() * characterizationSpeed.getRadians()));

        // Get yaw and wheel positions
        accumGyroYawRads += MathUtil.angleModulus(gyroAngleSupplier.get().getRadians() - lastGyroYawRads);
        lastGyroYawRads = gyroAngleSupplier.get().getRadians();
        double averageWheelPosition = 0.0;
        Rotation2d[] wheelPositions = wheelDriveDistanceSupplier.get();
        for (int i = 0; i < 4; i++) {
            averageWheelPosition += Math.abs(wheelPositions[i].minus(startWheelPositions[i]).getRadians());
        }
        averageWheelPosition /= 4.0;

        currentEffectiveWheelRadius = (accumGyroYawRads * driveRadiusMeters) / averageWheelPosition;
        Logger.recordOutput("Drive/RadiusCharacterization/DrivePosition", averageWheelPosition);
        Logger.recordOutput("Drive/RadiusCharacterization/AccumGyroYawRads", accumGyroYawRads);
        Logger.recordOutput(
                "Drive/RadiusCharacterization/CurrentWheelRadiusInches",
                Units.metersToInches(currentEffectiveWheelRadius)
        );
    }

    @Override
    public void end(boolean interrupted) {
        onEnd.run();
        if (accumGyroYawRads <= Math.PI * 2.0) {
            System.out.println("Not enough data for characterization");
        }
        else {
            System.out.println(
                    "Effective Wheel Radius: "
                            + Units.metersToInches(currentEffectiveWheelRadius)
                            + " inches");
        }
    }

}