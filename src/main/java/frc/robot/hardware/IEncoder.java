package frc.robot.hardware;

import com.revrobotics.REVLibError;

public interface IEncoder {

    REVLibError setPosition(double position);

    REVLibError setPositionConversionFactor(double factor);

    REVLibError setVelocityConversionFactor(double factor);

    REVLibError setInverted(boolean inverted);

}
