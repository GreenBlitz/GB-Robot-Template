package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IMotor {

    void setBrake(boolean brake);

    void resetAngle(Rotation2d angle);

    void setVoltage(double voltage);

    void setTargetVelocity(Rotation2d targetVelocity, ControlState controlState);

    void setTargetAngle(Rotation2d targetAngle, ControlState controlState);

    void setTargetAngle(Rotation2d targetAngle, ControlState controlState, int pidSlot);

    void updateInputs(MotorInputs motorInputs);

}
