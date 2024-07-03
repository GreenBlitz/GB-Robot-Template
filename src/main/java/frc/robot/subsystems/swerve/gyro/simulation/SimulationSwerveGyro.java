package frc.robot.subsystems.swerve.gyro.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.simulation.GyroSimulation;
import frc.robot.subsystems.swerve.gyro.gyrointerface.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.gyrointerface.SwerveGyroInputsAutoLogged;
import frc.utils.cycletimeutils.CycleTimeUtils;

public class SimulationSwerveGyro implements ISwerveGyro {

    private final GyroSimulation gyro = new GyroSimulation();

    @Override
    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading);
    }

    @Override
    public void updateInputs(SwerveGyroInputsAutoLogged inputs) {
        gyro.updateYaw(
                Rotation2d.fromRadians(RobotContainer.SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond),
                CycleTimeUtils.getCurrentCycleTime()
        );

        inputs.gyroYaw = gyro.getGyroYaw();
        inputs.odometryUpdatesYaw = new Rotation2d[]{inputs.gyroYaw};
        inputs.odometryUpdatesTimestamp = new double[]{Timer.getFPGATimestamp()};
    }

}
