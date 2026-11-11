package frc.robot.subsystems.arm;

import frc.utils.angle2d.Angle2d;
import frc.utils.utilcommands.InitExecuteCommand;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.RunCommand;

public class DynamicMotionMagicArmCommandsBuilder extends ArmCommandsBuilder {

  private final DynamicMotionMagicArm arm;

  protected DynamicMotionMagicArmCommandsBuilder(DynamicMotionMagicArm arm) {
    super(arm);
    this.arm = arm;
  }

  @Override
  public Command setTargetPosition(Angle2d position) {
    return arm.asSubsystemCommand(
        new InitExecuteCommand(() -> arm.setTargetPosition(position), () -> {}),
        "Set target position to: " + position);
  }

  public Command setTargetPosition(Supplier<Angle2d> position) {
    return arm.asSubsystemCommand(
        new RunCommand(() -> arm.setTargetPosition(position.get())),
        "Set target position by supplier");
  }

  public Command setTargetPosition(
      Angle2d position, Angle2d maxVelocityAPS, Angle2d maxAccelerationAPSSquared) {
    return arm.asSubsystemCommand(
        new InitExecuteCommand(
            () -> {},
            () -> arm.setTargetPosition(position, maxVelocityAPS, maxAccelerationAPSSquared)),
        "Set target position with "
            + maxAccelerationAPSSquared
            + " acceleration per Second Squared and with "
            + maxVelocityAPS
            + " velocity per second to:"
            + position);
  }
}
