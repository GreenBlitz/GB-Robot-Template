package frc.utils;

import java.io.IOException;

public class LaptopRunner {

    private static Runtime rt = Runtime.getRuntime();

    public static void main(String[] args) {

    }

    public static void openClassOnLaptop(Class classOfThing) {
        String nameOfClass = classOfThing.getName();
        nameOfClass = nameOfClass.substring(nameOfClass.lastIndexOf('.') + 1);

        String filePath = classOfThing.getResource(".").getPath().substring(1) + nameOfClass + ".java";

        filePath = filePath.replace("build/classes/java/main", "src/main/java");

        filePath = filePath.replace('/', '\\');


        System.out.println(filePath);
        try {
            rt.exec(new String[]{"cmd.exe","/c","java " + filePath});
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
