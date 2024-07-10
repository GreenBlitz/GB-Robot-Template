package frc.robot.subsystems.swerve.gyro.simulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.simulation.GyroSimulation;
import frc.robot.subsystems.swerve.gyro.ISwerveGyro;
import frc.robot.subsystems.swerve.gyro.SwerveGyroInputsAutoLogged;
import frc.utils.cycletime.CycleTimeUtils;

public class SimulationSwerveGyro implements ISwerveGyro {

    private final GyroSimulation gyro;

    public SimulationSwerveGyro(){
        this.gyro = new GyroSimulation();
    }

    @Override
    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading);
    }

    @Override
    public void updateInputs(SwerveGyroInputsAutoLogged inputs) {
        // todo: fix swerve getter
        gyro.updateYaw(
                Rotation2d.fromRadians(Robot.swerve.getRobotRelativeVelocity().omegaRadiansPerSecond),
                CycleTimeUtils.getCurrentCycleTime()
        );

        inputs.gyroYaw = gyro.getGyroYaw();
        inputs.yawOdometrySamples = new Rotation2d[]{inputs.gyroYaw};
        inputs.timestampOdometrySamples = new double[]{Timer.getFPGATimestamp()};
    }

}
