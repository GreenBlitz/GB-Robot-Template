package frc.robot.subsystems.swerve.modules.check.steer.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

public class TalonFXSteerConfigObject {

    private final TalonFXWrapper steerMotor;

    private final TalonFXSteerSignals signals;

    public TalonFXSteerConfigObject(CTREDeviceID motorID, int encoderID, TalonFXConfiguration configuration){
        this.steerMotor = new TalonFXWrapper(motorID);
        this.signals = new TalonFXSteerSignals(
                steerMotor.getPosition().clone(),
                steerMotor.getVelocity().clone(),
                steerMotor.getAcceleration().clone(),
                steerMotor.getMotorVoltage().clone()
        );

        configMotor(encoderID, configuration);
        optimizeBusAndSignalOfSteerMotor();
    }

    private void configMotor(int encoderID, TalonFXConfiguration steerConfiguration) {
        if (encoderID != TalonFXSteerConstants.NO_ENCODER_ID) {
            steerConfiguration.Feedback.FeedbackRemoteSensorID = encoderID;
        }
        steerMotor.applyConfiguration(steerConfiguration);
    }
    private void optimizeBusAndSignalOfSteerMotor() {
        BaseStatusSignal.setUpdateFrequencyForAll(
                PoseEstimatorConstants.ODOMETRY_FREQUENCY_HERTZ,
                signals.positionSignal(),
                signals.velocitySignal(),
                signals.accelerationSignal()
        );
        BaseStatusSignal.setUpdateFrequencyForAll(GlobalConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ, signals.voltageSignal());

        steerMotor.optimizeBusUtilization();
    }


    public TalonFXWrapper getSteerMotor() {
        return steerMotor;
    }
    public TalonFXSteerSignals getSignals() {
        return signals;
    }

}
