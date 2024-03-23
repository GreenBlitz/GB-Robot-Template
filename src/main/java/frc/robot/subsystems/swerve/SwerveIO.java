package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.falconswerve.FalconSwerve;
import frc.utils.RobotTypeUtils;
import org.littletonrobotics.junction.AutoLog;

public class SwerveIO {
    static SwerveIO generateIO() {
        return switch (RobotTypeUtils.getRobotType()) {
            default -> new FalconSwerve();
        };
    }

    protected void updateInputs(SwerveInputsAutoLogged inputs) {
    }

    protected void setHeading(Rotation2d heading) {
    }

    @AutoLog
    protected static class SwerveInputs {
        public double[] odometryUpdatesYawDegrees = new double[0];
        public double gyroYawDegrees = 0;
        public double gyroPitchDegrees = 0;
        public double accelerationX = 0;
        public double accelerationY = 0;
        public double accelerationZ = 0;

        public double[] odometryUpdatesTimestamp = new double[0];
    }
}
