package frc.robot;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Phoenix6Constants;
import frc.robot.constants.Ports;
import frc.utils.GBSubsystem;
import frc.utils.calibration.staticcharacterization.StaticCharacterizationObject;
import frc.utils.devicewrappers.GBTalonFXPro;
import frc.utils.joysticks.SmartJoystick;

public class JoysticksBindings {

    private static final SmartJoystick MAIN_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.MAIN);

    private static final SmartJoystick SECOND_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.SECOND);

    private static final SmartJoystick THIRD_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.THIRD);

    private static final SmartJoystick FOURTH_JOYSTICK = new SmartJoystick(Ports.JoystickDriverStationPorts.FOURTH);

    public static void configureBindings() {
        mainJoystickButtons();
        secondJoystickButtons();
        thirdJoystickButtons();
        fourthJoystickButtons();
    }

    static GBTalonFXPro motor = new GBTalonFXPro(0, Phoenix6Constants.CANIVORE_NAME);

    static {
        motor.applyConfiguration(new TalonFXConfiguration().withFeedback(
                new FeedbackConfigs().withSensorToMechanismRatio(150 / 7.0)));
    }

    static StaticCharacterizationObject staticCharacterizationObject = new StaticCharacterizationObject(
            new GBSubsystem() {
                @Override
                public void periodic() {
                    super.periodic();
                }
            },
            voltage -> motor.setControl(new VoltageOut(voltage)),
            () -> motor.getVelocity().refresh().getValue()
    );

    private static void mainJoystickButtons() {
        SmartJoystick usedJoystick = MAIN_JOYSTICK;
        // bindings
        usedJoystick.A.onTrue(staticCharacterizationObject.getFindKsKgCommand());
        usedJoystick.B.onTrue(new InstantCommand(() -> motor.setControl(new VoltageOut(0.24043274000))));
        usedJoystick.X.onTrue(new InstantCommand(() -> motor.setControl(new VoltageOut(0.1878287699))));
    }

    private static void secondJoystickButtons() {
        SmartJoystick usedJoystick = SECOND_JOYSTICK;
        // bindings
    }

    private static void thirdJoystickButtons() {
        SmartJoystick usedJoystick = THIRD_JOYSTICK;
        // bindings
    }

    private static void fourthJoystickButtons() {
        SmartJoystick usedJoystick = FOURTH_JOYSTICK;
        // bindings
    }

}
