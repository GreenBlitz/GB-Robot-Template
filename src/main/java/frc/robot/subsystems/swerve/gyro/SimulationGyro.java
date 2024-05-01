package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.simulation.GyroSimulation;
import frc.utils.roborioutils.RoborioUtils;

public class SimulationGyro implements IGyro {

    private final GyroSimulation gyro = new GyroSimulation();

    @Override
    public void setHeading(Rotation2d heading) {
        gyro.setYaw(heading);
    }

    @Override
    public void updateInputs(GyroInputsAutoLogged inputs) {
        gyro.updateYaw(
                Rotation2d.fromRadians(RobotContainer.SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond),
                RoborioUtils.getCurrentRoborioCycleTime()
        );
        inputs.gyroYaw = gyro.getGyroYaw().getDegrees();
        inputs.odometryUpdatesYawDegrees = new double[]{inputs.gyroYaw};
        inputs.odometryUpdatesTimestamp = new double[]{Timer.getFPGATimestamp()};
    }

}
