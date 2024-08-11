package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.MathConstants;

public class HeadingStabilizer {

    private final PIDController headingController;

    public HeadingStabilizer(SwerveConstants constants) {
        this.headingController = new PIDController(
                constants.rotationDegreesPIDController().getP(),
                constants.rotationDegreesPIDController().getI(),
                constants.rotationDegreesPIDController().getD()
        );
        headingController.enableContinuousInput(-MathConstants.HALF_CIRCLE.getDegrees(), MathConstants.HALF_CIRCLE.getDegrees());
    }

    public void setHeadingSetpoint(Rotation2d headingSetpoint){
        headingController.reset();
        headingController.setSetpoint(headingSetpoint.getDegrees());
    }

    public Rotation2d calculate(Rotation2d currentHeading){
        return Rotation2d.fromDegrees(headingController.calculate(currentHeading.getDegrees()));
    }

}
