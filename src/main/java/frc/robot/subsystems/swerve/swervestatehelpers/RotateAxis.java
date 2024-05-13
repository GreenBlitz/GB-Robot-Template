package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.modules.ModuleUtils;

public enum RotateAxis {

    MIDDLE_OF_ROBOT(0, 0),
    FRONT_LEFT_MODULE(ModuleUtils.getModulePositionRelativeToMiddleOfRobot(ModuleUtils.ModuleName.FRONT_LEFT)),
    FRONT_RIGHT_MODULE(ModuleUtils.getModulePositionRelativeToMiddleOfRobot(ModuleUtils.ModuleName.FRONT_RIGHT)),
    BACK_LEFT_MODULE(ModuleUtils.getModulePositionRelativeToMiddleOfRobot(ModuleUtils.ModuleName.BACK_LEFT)),
    BACK_RIGHT_MODULE(ModuleUtils.getModulePositionRelativeToMiddleOfRobot(ModuleUtils.ModuleName.BACK_RIGHT));

    private final Translation2d rotateAxis;

    RotateAxis(double x, double y) {
        this.rotateAxis = new Translation2d(x, y);
    }

    RotateAxis(Translation2d rotateAxis) {
        this.rotateAxis = new Translation2d(rotateAxis.getX(), rotateAxis.getY());
    }

    public Translation2d getRotateAxis() {
        return new Translation2d(rotateAxis.getX(), rotateAxis.getY());
    }

    public static RotateAxis getLeftFarRotateAxis() {
        Rotation2d currentAllianceAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation2d().getAllianceAngle();
        if (Math.abs(currentAllianceAngle.getDegrees()) <= 45) { // -45 <= deg <= 45
            return FRONT_LEFT_MODULE;
        }
        if (Math.abs(currentAllianceAngle.getDegrees()) >= 135) {// -135 - x - 135
            return BACK_RIGHT_MODULE;
        }
        if (currentAllianceAngle.getDegrees() > 0) {// 45 <= x <= 135
            return FRONT_RIGHT_MODULE;
        }
        return BACK_LEFT_MODULE;// -45 >= x >= -135
    }

    public static RotateAxis getRightFarRotateAxis() {
        Rotation2d currentAllianceAngle = RobotContainer.POSE_ESTIMATOR.getCurrentPose().getRotation2d().getAllianceAngle();
        if (Math.abs(currentAllianceAngle.getDegrees()) <= 45) { // -45 <= deg <= 45
            return FRONT_RIGHT_MODULE;
        }
        if (Math.abs(currentAllianceAngle.getDegrees()) >= 135) {// -135 - x - 135
            return BACK_LEFT_MODULE;
        }
        if (currentAllianceAngle.getDegrees() > 0) {// 45 <= x <= 135
            return BACK_RIGHT_MODULE;
        }
        return FRONT_LEFT_MODULE;// -45 >= x >= -135
    }
}
