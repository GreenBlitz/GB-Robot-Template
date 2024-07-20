package frc.robot.subsystems.swerve.swervecontainer.simulation;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;

public class ConstantsContainer {

    protected static final SwerveConstants swerveConstants = new SwerveConstants(
            SimulationSwerve.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
            Rotation2d.fromRadians(10),
            new PIDConstants(7, 0, 0),
            new PIDConstants(6, 0, 0)
    );

}
