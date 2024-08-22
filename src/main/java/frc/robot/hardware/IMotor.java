package frc.robot.hardware;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IMotor {

    void setBrake(boolean brake);

    void resetAngle(Rotation2d angle);

    void setVoltage(double voltage);

    void setTargetVelocity(VelocityControl velocityControl);

    void setTargetPosition(PositionControl positionControl);

    void updateInputs(MotorInputs motorInputs);

}
