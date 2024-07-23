package frc.robot.subsystems.swerve.constants;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.SwerveGyroConstants;
import frc.robot.subsystems.swerve.modules.simulation.SimulationModuleConstants;

public class SimulationSwerveConstants {

    private static final double VELOCITY_AT_12_VOLTS_METERS_PER_SECOND = 5.054;

    private static final Rotation2d MAX_ROTATION_VELOCITY = Rotation2d.fromRadians(10);

    private static final PIDConstants TRANSLATION_METERS_PID_CONSTANTS = new PIDConstants(7, 0, 0);
    private static final PIDConstants ROTATION_DEGREES_PID_CONSTANTS = new PIDConstants(6, 0, 0);

    protected static final SwerveConstants swerveConstants = new SwerveConstants(
            VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
            MAX_ROTATION_VELOCITY,
            TRANSLATION_METERS_PID_CONSTANTS,
            ROTATION_DEGREES_PID_CONSTANTS,
            new SimulationModuleConstants[]{
                    SimulationModulesConstants.getModuleConstants(VELOCITY_AT_12_VOLTS_METERS_PER_SECOND),
                    SimulationModulesConstants.getModuleConstants(VELOCITY_AT_12_VOLTS_METERS_PER_SECOND),
                    SimulationModulesConstants.getModuleConstants(VELOCITY_AT_12_VOLTS_METERS_PER_SECOND),
                    SimulationModulesConstants.getModuleConstants(VELOCITY_AT_12_VOLTS_METERS_PER_SECOND)
            },
            new SwerveGyroConstants()
    );

}
