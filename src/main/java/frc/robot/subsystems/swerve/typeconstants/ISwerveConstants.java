package frc.robot.subsystems.swerve.typeconstants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;

public interface ISwerveConstants {

    double getMaxSpeedMetersPerSecond();

    Rotation2d getMaxRotationSpeedPerSecond();

    PIDConstants getTranslationPIDConstants();

    PIDConstants getRotationPIDConstants();

}
