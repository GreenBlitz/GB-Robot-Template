package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.constants.MathConstants;

public enum RotateAxis {

    MIDDLE_OF_ROBOT(),
    FRONT_LEFT_MODULE(),
    FRONT_RIGHT_MODULE(),
    BACK_LEFT_MODULE(),
    BACK_RIGHT_MODULE();

    public static RotateAxis getLeftFarRotateAxis() {
        return getFarRotateAxis(true);
    }

    public static RotateAxis getRightFarRotateAxis() {
        return getFarRotateAxis(false);
    }

    private static RotateAxis getFarRotateAxis(boolean isLeft) {
        Rotation2d currentAllianceAngle = Robot.swerve.getAllianceRelativeHeading();
        if (Math.abs(currentAllianceAngle.getDegrees()) <= MathConstants.EIGHTH_CIRCLE.getDegrees()) { // -45 <= x <= 45
            return isLeft ? FRONT_LEFT_MODULE : FRONT_RIGHT_MODULE;
        }
        if (Math.abs(currentAllianceAngle.getDegrees()) >= MathConstants.EIGHTH_CIRCLE.getDegrees() * 3) { // -135 - x - 135
            return isLeft ? BACK_RIGHT_MODULE : BACK_LEFT_MODULE;
        }
        if (currentAllianceAngle.getDegrees() > 0) { // 45 <= x <= 135
            return isLeft ? FRONT_RIGHT_MODULE : BACK_RIGHT_MODULE;
        }
        return isLeft ? BACK_LEFT_MODULE : FRONT_LEFT_MODULE; // -45 >= x >= -135
    }

}
