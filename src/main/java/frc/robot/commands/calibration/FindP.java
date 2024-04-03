package frc.robot.commands.calibration;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utils.commands.GBCommand;
import frc.utils.devicewrappers.GBTalonFXPro;

public class FindP extends GBCommand {

    private final GBTalonFXPro motor;
    private final PositionVoltage controlRequest;
    private final double targetValue1;
    private final double targetValue2;
    private double usedTargetValue;
    private final Slot0Configs slot0Configs;

    private final double tolerance;

    private final Timer timer;
    private final double timeout;

    private final double changePFactor;

    private final double minErrorRange;
    private final double maxErrorRange;

    private boolean isCheckingMin;
    private double edgeValue;

    
    private double accuracy;

    private boolean isInit;
    private boolean isExe;
    private boolean isEnd;

    //Todo - add motor sub sys with SYSID and the functions: getVelocity, setVelocity, getPosition, setPosition, getPValue, setPValue
    //Todo - instead of getting motor and control get motorSubSys

    public FindP(GBTalonFXPro motor, PositionVoltage controlRequest, double targetValue1, double targetValue2, double timeout, double tolerance, double factor, double startRange, double endRange) {
        this.motor = motor;
        this.controlRequest = controlRequest;
        
        this.targetValue1 = targetValue1;
        this.targetValue2 = targetValue2;
        
        this.slot0Configs = new Slot0Configs();
        motor.getConfigurator().refresh(slot0Configs);

        this.timer = new Timer();
        this.timeout = timeout;

        this.minErrorRange = startRange;
        this.maxErrorRange = endRange;

        this.changePFactor = factor;
        this.tolerance = tolerance;

        this.usedTargetValue = targetValue2;

        this.accuracy = 0;
        
        this.isInit = true;
        this.isExe = false;
        this.isEnd = false;
    }

    public double getTargetValue(double lastUsedTargetValue) {
        return lastUsedTargetValue == targetValue2 ? targetValue1 : targetValue2;
    }

    private void setIsInit(boolean isInit){
        this.isInit = isInit;
        if (isInit){
            isEnd = false;
            isExe = false;
        }
    }
    private void setIsExe(boolean isExe){
        this.isExe = isExe;
        if (isExe){
            isInit = false;
            isEnd = false;
        }
    }
    private void setIsEnd(boolean isEnd){
        this.isEnd = isEnd;
        if (isEnd){
            isInit = false;
            isExe = false;
        }
    }
    
    @Override
    public void initialize() {
        motor.setPosition(0);
    }
    
    @Override
    public void execute() {
        if (isInit) {
            double curPosition = motor.getPosition().refresh().getValue();
            accuracy = 0;
            timer.restart();

            usedTargetValue = getTargetValue(usedTargetValue);
            isCheckingMin = curPosition > usedTargetValue;
            edgeValue = curPosition;
            motor.setControl(controlRequest.withPosition(usedTargetValue));

            setIsExe(true);
        }

        else if (isExe) {
            double curPosition = motor.getPosition().refresh().getValue();
            SmartDashboard.putNumber("curPosw", curPosition);
            if (isCheckingMin) {
                if (edgeValue > curPosition) {
                    edgeValue = curPosition;
                }
            } else {
                if (edgeValue < curPosition) {
                    edgeValue = curPosition;
                }
            }
            if (Math.abs(usedTargetValue - curPosition) <= tolerance || timer.hasElapsed(timeout)){
                setIsEnd(true);
            }
        }

        else if (isEnd) {
            timer.stop();

            double sign = isCheckingMin ? Math.signum(edgeValue - usedTargetValue) : Math.signum(usedTargetValue - edgeValue);
            double error = Math.abs(edgeValue - usedTargetValue);
            
            SmartDashboard.putNumber("ERROR", error);
            
            accuracy = 100 - (100 / (maxErrorRange - minErrorRange + 1)) * error;
            SmartDashboard.putNumber("ACCURACY", accuracy);
            if (accuracy < 95){
                motor.getConfigurator().refresh(slot0Configs);
                slot0Configs.kP += sign * error / changePFactor;
                SmartDashboard.putNumber("kp calc",sign * error / changePFactor);
                SmartDashboard.putNumber("kp", slot0Configs.kP);
                motor.getConfigurator().apply(slot0Configs);
                setIsInit(true);
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        return accuracy > 95;
    }
    
    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }
}
