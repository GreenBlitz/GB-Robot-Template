package frc.utils;

import frc.utils.controllers.keyboard.KeyboardListener;

import java.io.IOException;

public class LaptopRunner {

    private static Runtime rt = Runtime.getRuntime();

    public static void deployWithKeyboard() {
        openClassOnLaptop(KeyboardListener.class);
        deployGradle();
    }

    public static void main(String[] args) {
        deployGradle();
    }
    public static void deployGradle() {
        runCMDCommand("gradle deploy");// TODO make this work
    }

    public static void openClassOnLaptop(Class classOfThing) {
        String nameOfClass = classOfThing.getName();
        nameOfClass = nameOfClass.substring(nameOfClass.lastIndexOf('.') + 1);

        String filePath = classOfThing.getResource(".").getPath().substring(1) + nameOfClass + ".java";

        filePath = filePath.replace("build/classes/java/main", "src/main/java");

        filePath = filePath.replace('/', '\\');

        runCMDCommand("java" + filePath);
    }

    public static void runCMDCommand(String command) {
        try {
            rt.exec(new String[]{"cmd.exe","/c",command});
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
