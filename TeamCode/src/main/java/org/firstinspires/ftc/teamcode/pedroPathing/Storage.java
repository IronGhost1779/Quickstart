package org.firstinspires.ftc.teamcode.pedroPathing;

public final class Storage {
    private Storage() {}

    public static boolean[] lista_MOR = new boolean[] { false, false, false };
    public static boolean valid = false;

    public static float[] lista_PINPOINT = new float[] { 56, 8, 90 };
    public static boolean valid1 = false;
    public static void set(boolean a, boolean b, boolean c) {
        lista_MOR[0] = a;
        lista_MOR[1] = b;
        lista_MOR[2] = c;
        valid = true;
    }

    public static void set1(float a, float b, float c){
        lista_PINPOINT[0] = a;
        lista_PINPOINT[1] = b;
        lista_PINPOINT[2] = c;
        valid1 = true;
    }
}
