package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;

public class StatesMotionPlanner {

    private final Superstructure superstructure;

    public StatesMotionPlanner(Superstructure superstructure) {
        this.superstructure = superstructure;
    }

    public Command setState(RobotState state) {
        return switch (state) {
            case INTAKE ->
                superstructure.intake().until(superstructure::isEnableChangeStateAutomatically).andThen(superstructure.idle());
            case ARM_INTAKE ->
                superstructure.armIntake().until(superstructure::isEnableChangeStateAutomatically).andThen(superstructure.idle());
            case SPEAKER ->
                superstructure.speaker().until(superstructure::isEnableChangeStateAutomatically).andThen(superstructure.idle());
            case TRANSFER_SHOOTER_TO_ARM ->
                superstructure.transferShooterToArm().until(superstructure::isEnableChangeStateAutomatically).andThen(superstructure.idle());
            case TRANSFER_ARM_TO_SHOOTER ->
                superstructure.transferArmToShooter().until(superstructure::isEnableChangeStateAutomatically).andThen(superstructure.idle());
            case INTAKE_OUTTAKE, AMP, ARM_OUTTAKE, PRE_AMP, PRE_SPEAKER, IDLE -> superstructure.setState(state);
        };
    }

}
