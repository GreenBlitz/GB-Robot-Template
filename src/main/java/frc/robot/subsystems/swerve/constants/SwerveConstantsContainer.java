package frc.robot.subsystems.swerve.constants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;

public class SwerveConstantsContainer {

    protected static final SwerveConstants mk4iSwerveConstants = new SwerveConstants(
        5.052,
            Rotation2d.fromRadians(10),
            new PIDConstants(6,0,0),
            new PIDConstants(6,0,0)
    );

    protected static final SwerveConstants simulationSwerveConstants = new SwerveConstants(
            5.052,
            Rotation2d.fromRadians(10),
            new PIDConstants(6,0,0),
            new PIDConstants(6,0,0)
    );

}
