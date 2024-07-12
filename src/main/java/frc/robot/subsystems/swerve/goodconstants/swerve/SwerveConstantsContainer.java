package frc.robot.subsystems.swerve.goodconstants.swerve;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveConstantsContainer {

    protected static final SwerveConstantsObject mk4iSwerveConstants = new SwerveConstantsObject(
        5.052,
            Rotation2d.fromRadians(10),
            new PIDConstants(6,0,0),
            new PIDConstants(6,0,0)
    );

    protected static final SwerveConstantsObject simulationSwerveConstants = new SwerveConstantsObject(
            5.052,
            Rotation2d.fromRadians(10),
            new PIDConstants(6,0,0),
            new PIDConstants(6,0,0)
    );

}
