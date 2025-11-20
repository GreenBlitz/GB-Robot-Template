package frc.constants.fourBar;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.geometry.Rotation2d;

public class FourBarConstants {
    public static final String LOG_PATH;
    public static final int CURRENT_LIMIT = 40;
    public static final double MOMENT_OF_INERTIA = 0.0001;
    public static final double ARM_LENGTH_METERS = 0.3;
    public static final Rotation2d FORWARD_SOFTWARE_LIMITS;
    public static final Rotation2d BACKWARD_SOFTWARE_LIMITS;
    public static final Rotation2d MAX_ACCELERATION_ROTATION2D_METERS_PER_SECONDS_SQUARE;
    public static final Rotation2d MAX_VELOCITY_ROTATION2D_METERS_PER_SECONDS;
    public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
    public static final Slot0Configs REAL_SLOTS_CONFIGS = new Slot0Configs();
    public static final Slot0Configs SIMULATION_SLOTS_CONFIGS = new Slot0Configs();
}
