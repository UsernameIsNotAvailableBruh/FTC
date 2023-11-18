package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;

public class implementingActions {

    static ArrayList<ActionPrev> buildFirstMethod(Motor myMotor) {
        System.err.println("hi alex, build the old fashion way");
        ActionPrev setMotorTo2ActionPrev = new ActionPrev("Set motor to 2", () -> {
            myMotor.setTargetPosition(2);
            return true;
        });
        ActionFunction myWait = () -> { return myMotor.getCurrentPosition() >= 2; };
        ActionPrev waitMotorAt2 = new ActionPrev("Wait until 2", myWait);
        ArrayList<ActionPrev> stuff = new ArrayList<ActionPrev>();
        stuff.add(setMotorTo2ActionPrev);
        stuff.add(waitMotorAt2);
        return stuff;
    }

    /*static ArrayList<org.firstinspires.ftc.teamcode.Action> buildSecondMethod(Motor myMotor, Timer myTimer) {
        System.err.println("hi alex, build the new fashion way");
        return new ActionBuilder()
                .resetTimer(myTimer)
                .setMotorPosition(myMotor, 3)
                .waitForMotorAbovePosition(myMotor, 2)
                .waitUntil(myTimer, 4)
                .getList();
    }*/

//    public static void main(String[] args) {
//        // Write your code here
//        System.out.println("Hello alex!");
//        Motor myMotor = new Motor(hardwareMap, "lift", 0);
//        Timer myTimer = new Timer("time to lunch");
//        ArrayList<org.firstinspires.ftc.teamcode.Action> stuff;
//        if (false)
//            stuff = buildFirstMethod(myMotor);
//        else
//            stuff = buildSecondMethod(myMotor, myTimer);
//
//        int i = 0;
//        for (int step = 0; i < stuff.size(); step++) {
//            System.err.println("Step " + step + " with stuff action " + i);
//            if (stuff.get(i).performAction()) {
//                System.err.println("  SUCCESS, go to next stuff");
//                i++;
//            } else {
//                System.err.println("  failure, go on");
//            }
//            myMotor.updatePosition();
//            myTimer.updateTime();
//        }
//        System.err.println("Bravo Alex, we are done");
//    }
}

