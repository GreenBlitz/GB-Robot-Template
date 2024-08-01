package frc.robot.subsystems.swerve.modules.steer.talonfx;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import frc.robot.constants.GlobalConstants;
import frc.robot.poseestimation.PoseEstimatorConstants;
import frc.utils.ctre.CTREDeviceID;
import frc.utils.devicewrappers.TalonFXWrapper;

class TalonFXSteerConfigObject {

    private final TalonFXWrapper steerMotor;

    private final TalonFXSteerSignals signals;

    protected TalonFXSteerConfigObject(CTREDeviceID motorID, boolean inverted, int encoderID, TalonFXConfiguration configuration){
        this.steerMotor = new TalonFXWrapper(motorID);
        this.signals = new TalonFXSteerSignals(
                steerMotor.getPosition().clone(),
                steerMotor.getVelocity().clone(),
                steerMotor.getAcceleration().clone(),
                steerMotor.getMotorVoltage().clone()
        );

        configMotor(encoderID, configuration);
        steerMotor.setInverted(inverted);
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


    protected TalonFXWrapper getSteerMotor() {
        return steerMotor;
    }
    protected TalonFXSteerSignals getSignals() {
        return signals;
    }

}
