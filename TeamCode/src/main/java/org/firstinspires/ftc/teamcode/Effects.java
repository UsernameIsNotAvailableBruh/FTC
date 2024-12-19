package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Effects {
    public Gamepad.LedEffect.Builder RGBGradient(double AddValue, int DurationMs){
        Gamepad.LedEffect.Builder Led = new Gamepad.LedEffect.Builder();
        // R to G
        for ( double i=0;i<=1;i+=AddValue) { //R to Black
            double RoundedI = (Math.round(i * 100) / 100.0); //turns something like 3.00000004 into 3.0
            Led = Led.addStep(1 - RoundedI, 0, 0, DurationMs);
        }
        for (double i=0;i<=1;i+=AddValue) { //Black to G
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(0, RoundedI, 0, DurationMs);
        }
        // G to B
        for (double i=0;i<=1;i+=AddValue) { //G to Black
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(0, 1 - RoundedI, 0, DurationMs);
        }
        for (double i=0;i<=1;i+=AddValue) { //Black to B
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(0, 0, RoundedI, DurationMs);
        }
        // B to R
        for (double i=0;i<=1;i+=AddValue) { //B to Black
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(0, 0, 1 - RoundedI, DurationMs);
        }
        for (double i=0;i<=1;i+=AddValue) { //Black to R
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(RoundedI, 0, 0, DurationMs);
        }
        return Led;
    }

    public Gamepad.RumbleEffect.Builder RumbleBothMotorsOpp() {
        Gamepad.RumbleEffect.Builder rumble = new Gamepad.RumbleEffect.Builder();
        for (double i = 0; i < 1; i += .1) {
            rumble = rumble.addStep(i, 1 - i, 100);
            rumble = rumble.addStep(1 - i, i, 100);
        }
        return rumble;
    }

}
