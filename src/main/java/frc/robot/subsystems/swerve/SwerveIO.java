package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.swerve.falconswerve.FalconSwerve;
import frc.utils.RobotTypeUtils;

public class SwerveIO {
    static SwerveIO generateIO() {
        return switch (RobotTypeUtils.getRobotType()) {
            default -> new FalconSwerve();
        };
    }

    protected void setHeading(Rotation2d heading) {
    }

    protected void updateInputs(SwerveInputsAutoLogged inputs) {
    }

}
