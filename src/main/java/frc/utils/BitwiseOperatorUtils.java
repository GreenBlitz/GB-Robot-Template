package frc.utils;

import static java.lang.Math.pow;

public class BitwiseOperatorUtils {


    /**
     * @param number positive integer
     * @return an array of the binary of hte number or whatever
     * <p>
     * for example:
     * input: 4. output: [1, 0, 0] (array from left to right)
     * input: 8. output: [1, 0, 0, 0] (array from left to right)
     */
    public static boolean[] convertNumberToBinaryArray(int number) {
        String binaryString = Integer.toBinaryString(number);
        boolean[] binaryArray = new boolean[binaryString.length()];

        for (int i = 0; i < binaryString.length(); i++) {
            binaryArray[i] = binaryString.charAt(i) != '0';
        }
        return binaryArray;
    }

    public static int convertBinaryArrayToInteger(boolean[] binaryArray) {
        int sum = 0;
        for (int i = binaryArray.length - 1; i >= 0; i--) {
            sum += (int) (binaryArray[i] ? 1 * pow(2,binaryArray.length - i - 1) : 0);
        }
        return sum;
    }
}
