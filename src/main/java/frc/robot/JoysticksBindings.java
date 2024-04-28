package frc.robot;

import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.Ports;
import frc.robot.simulation.SingleJointedArmSimulation;
import frc.utils.GBSubsystem;
import frc.utils.calibration.staticcharacterization.StaticCharacterizationObject;
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

    static SingleJointedArmSimulation armSimulation = new SingleJointedArmSimulation(
            DCMotor.getFalcon500Foc(1),
            20,
            1,
            1,
            Rotation2d.fromRotations(0),
            Rotation2d.fromRotations(1000),
            Rotation2d.fromRotations(0),
            true
    );

    static class Sub extends GBSubsystem {

    }

    static StaticCharacterizationObject staticCharacterizationObject = new StaticCharacterizationObject(
            new Sub(),
            voltage -> armSimulation.setControl(new VoltageOut(voltage)),
            () -> armSimulation.getVelocity().getRotations()
    );

    private static void mainJoystickButtons() {
        SmartJoystick usedJoystick = MAIN_JOYSTICK;
        // bindings
        usedJoystick.A.onTrue(staticCharacterizationObject.getFindKsKgCommand());
        usedJoystick.X.onTrue(new InstantCommand(() -> armSimulation.setControl(new VoltageOut(0.48))));
        usedJoystick.Y.onTrue(new InstantCommand(() -> armSimulation.setControl(new VoltageOut(1))));
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
