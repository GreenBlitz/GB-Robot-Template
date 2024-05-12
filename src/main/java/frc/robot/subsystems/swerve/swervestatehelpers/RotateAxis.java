package frc.robot.subsystems.swerve.swervestatehelpers;

import edu.wpi.first.math.geometry.Translation2d;
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
}
