package frc.robot.subsystems.swerve.simulationswerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.gyro.GyroSimulation;
import frc.robot.subsystems.swerve.swerveinterface.ISwerve;
import frc.robot.subsystems.swerve.swerveinterface.SwerveInputsAutoLogged;
import frc.utils.roborioutils.RoborioUtils;

public class SimulationSwerve implements ISwerve {

    private final GyroSimulation gyro = new GyroSimulation();

    @Override
    public void setHeading(Rotation2d heading) {
        gyro.setHeading(heading);
    }

    @Override
    public void updateInputs(SwerveInputsAutoLogged inputs) {
        gyro.update(RobotContainer.SWERVE.getSelfRelativeVelocity().omegaRadiansPerSecond, RoborioUtils.getCurrentRoborioCycle());
        inputs.gyroYawDegrees = gyro.getGyroYaw().getDegrees();
    }

}
