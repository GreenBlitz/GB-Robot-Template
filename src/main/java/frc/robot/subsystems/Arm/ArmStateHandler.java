package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmStateHandler {
    private final Arm arm;

    public ArmStateHandler(Arm arm){
        this.arm = arm;
    }

    public Command setState(ArmStates state){
        return arm.getCommandBuilder().moveToPosition(state.getPosition());
    }
}
