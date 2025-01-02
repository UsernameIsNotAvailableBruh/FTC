package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.internal.system.AppUtil.ROBOT_DATA_DIR;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class AutoReadData {
    static double LFm;
    static double LFb;

    static double RFm;
    static double RFb;

    static double LBm;
    static double LBb;

    static double RBm;
    static double RBb;

    public AutoReadData(String file) {
        Scanner FileScanner;
        try {
            FileScanner = new Scanner(new File(ROBOT_DATA_DIR, file));
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }
        String[][] Arr  = new String[][] {
            FileScanner.nextLine().split(" "), FileScanner.nextLine().split(" "), FileScanner.nextLine().split(" "), FileScanner.nextLine().split(" ")
        };

        LFm = Double.parseDouble(Arr[0][0]);
        LFb = Double.parseDouble(Arr[0][1]);

        RFm = Double.parseDouble(Arr[1][0]);
        RFb = Double.parseDouble(Arr[1][1]);

        LBm = Double.parseDouble(Arr[2][0]);
        LBb = Double.parseDouble(Arr[2][1]);

        RBm = Double.parseDouble(Arr[3][0]);
        RBb = Double.parseDouble(Arr[3][1]);

        FileScanner.close();
    }
}
