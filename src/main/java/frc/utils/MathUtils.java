package frc.utils;

public class MathUtils {

    public static double[] multiplyArrays(double[] array1, double[] array2) {
        double[] result = new double[Math.min(array1.length, array2.length)];
        for (int i = 0; i < result.length; i++) {
            result[i] = array1[i] * array2[i];
        }
        return result;
    }

}
