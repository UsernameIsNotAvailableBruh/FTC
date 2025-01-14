/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.internal.system.AppUtil.ROBOT_DATA_DIR;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

//NOTES:
//git fetch --all
//git reset --hard origin/master
//git pull

// *--*
// All the code is written by Aajinkya Naik unless stated otherwise.
// *--*

//BHI260AP is the IMU

//The name of the Driver Hub config file is "ILoveDickstein" (dedicated to my dearest friend, Jacob Dickstein (who's a captain of 10847))
@TeleOp(name="BuggyDicksteinsClaw", group="OpMode")
public class ClawRotation extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime           = new ElapsedTime();
    private DcMotor linearSlidey          = null;
    private Servo LeftServo               = null;
    private Servo RightServo              = null;
    //private Servo ExtendyServo            = null;
    //private Servo TurnServo               = null;
    private DcMotor actuatorDrive         = null;
    private DcMotor slideyTurni           = null;
    //private DistanceSensor REV2mDistance  = null;
    private final static double LowerBy = 1.5;
    @Override
    public void runOpMode() {
        //https://gm0.org/en/latest/docs/software/tutorials/gamepad.html#storing-gamepad-state
        //Gamepad currentGamepad1  = new Gamepad();
        //Gamepad previousGamepad1 = new Gamepad();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.

        linearSlidey = hardwareMap.get(DcMotor.class, "slidey");
        slideyTurni = hardwareMap.get(DcMotor.class, "slidey2");
        actuatorDrive = hardwareMap.get(DcMotor.class, "actorio");

        LeftServo = hardwareMap.get(Servo.class, "serv1"); //port 0 - the one closer to the linear slider
        RightServo = hardwareMap.get(Servo.class, "serv2"); //port 1
        //ExtendyServo = hardwareMap.get(Servo.class, "serv3");


        // Effects (just for funsies)
        Effects effects = new Effects();

        // rumble
        Gamepad.RumbleEffect.Builder rumble = effects.RumbleBothMotorsOpp();

        gamepad2.runRumbleEffect(rumble.build());
        gamepad1.runRumbleEffect(rumble.build());

        //LED effects (also for funsies)
        double AddValue = .01;
        int DurationMs = 10;
        Gamepad.LedEffect.Builder Led = effects.RGBGradient(AddValue, DurationMs);
        Led.setRepeating(true);
        gamepad1.runLedEffect(Led.build());

        // Wait for the game to start (driver presses START)

        telemetry.update();
        List<LynxModule> Hubs = hardwareMap.getAll(LynxModule.class); //I took this lynx thingy from gm0's bulk reads page https://gm0.org/en/latest/docs/software/tutorials/bulk-reads.html
        for (LynxModule hub : Hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // run until the end of the match (driver presses STOP)

        setZPBrake(false);
        boolean ZPFloatToggle = true;
        boolean ClawServoToggle = true;
        boolean ClawToggle = true;
        linearSlidey.setDirection(DcMotor.Direction.REVERSE);
        slideyTurni.setDirection(DcMotorSimple.Direction.FORWARD);

        linearSlidey.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideyTurni.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        linearSlidey.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideyTurni.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Buttons ButtonMonitor = new Buttons(true);
        double LowerPowerBy = 1;
        boolean LowerModeToggle = false;
        int LinearCurrentPos = 0;
        int SlideTurniCurrentPos = 0;
        LeftServo.setDirection(Servo.Direction.REVERSE);
        RightServo.setDirection(Servo.Direction.FORWARD);
        setZPFloat(true);
        LeftServo.setPosition(.5);
        RightServo.setPosition(.3);
        waitForStart();
        telemetry.addData("Status", "Initialized");
        runtime.reset();
        sleep(1000);
        while (opModeIsActive()) {
            ButtonMonitor.update();

            if (ButtonMonitor.Pressed(buttonName.left_stick_button))
                gamepad1.rumble(100);
            if (ButtonMonitor.Pressed(buttonName.right_stick_button))
                gamepad2.stopRumble();

            if (ButtonMonitor.wasPressed(buttonName.options)) {
                ZPFloatToggle = !ZPFloatToggle;
            }
            if (ZPFloatToggle) {
                setZPFloat(true);
            } else {
                setZPBrake(true);
            }

            if (ButtonMonitor.wasPressed(buttonName.share)){
                LowerModeToggle = !LowerModeToggle;
            }
            if (LowerModeToggle) {
                LowerPowerBy = LowerBy;
            }
            else {
                LowerPowerBy = 1;
            }

            if (ButtonMonitor.wasPressed(buttonName.cross)) {
                ClawServoToggle = !ClawServoToggle;
            }
            if (ClawServoToggle) {
                LeftServo.setPosition(.7);
                RightServo.setPosition(.7);
            }
            else if (!ClawServoToggle) {
                LeftServo.setPosition(.3);
                RightServo.setPosition(.3);
            }

//                if (ButtonMonitor.isPressed(buttonName.dpad_left) && ButtonMonitor.isPressed(buttonName.dpad_right)) {
//                    ExtendAmtPos = .5;
//                }
//                else if (ButtonMonitor.Pressed(buttonName.dpad_left)) {
//                    ExtendAmtPos = .999;
//                }
//                else if (ButtonMonitor.Pressed(buttonName.dpad_right)) {
//                    ExtendAmtPos = 0;
//                }

            final double GearRatio4 = 3.61;
            final double GearRatio5 = 5.23;
            final double DriveHDHexMotorCPR = 28 * GearRatio5 * GearRatio4;
            SlideTurniCurrentPos = slideyTurni.getCurrentPosition();
            double SlideTurniAngle = (SlideTurniCurrentPos / DriveHDHexMotorCPR) * 360;//no need to mod 360 (i think)

            if (slideyPostoAngle(slideyTurni.getCurrentPosition()) >= 90){
                ClawToggle = false;
            }
            else{
                ClawToggle = true;
            }
            slideyTurni.setPower(ClawToggle? .1:-.1);
            if (ButtonMonitor.isPressed(buttonName.left_bumper)) {
                slideyTurni.setPower(slideyTurni.getPower()+.45);
            }
            else if (ButtonMonitor.isPressed(buttonName.right_bumper)) {
                slideyTurni.setPower(slideyTurni.getPower()-.5);
            }

            double SlideyPower = -gamepad2.left_stick_y/LowerPowerBy;
            //ExtendyServo.setPosition(ExtendAmtPos);
            LinearCurrentPos = linearSlidey.getCurrentPosition();
            final int LinearSlideyLimit = 100_000; //ignore limit
            if (linearSlidey.getCurrentPosition() > LinearSlideyLimit) {
                linearSlidey.setTargetPosition(LinearSlideyLimit);
                linearSlidey.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlidey.setPower(.1);
            }
            else if (SlideyPower == 0) {
                linearSlidey.setTargetPosition(LinearCurrentPos);
                linearSlidey.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linearSlidey.setPower(.4);
            }
            else {
                if (linearSlidey.getMode() == DcMotor.RunMode.RUN_TO_POSITION){
                    linearSlidey.setTargetPosition(LinearSlideyLimit-10);
                }
                linearSlidey.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                int closeness = 250;
                if (linearSlidey.getCurrentPosition() >= LinearSlideyLimit-closeness) { // close to the top
                    linearSlidey.setPower(SlideyPower/LowerBy);
                }
                else {
                    linearSlidey.setPower(SlideyPower);
                }
            }

            actuatorDrive.setPower(0);
            if (ButtonMonitor.isPressed(buttonName.dpad_up)){
                actuatorDrive.setPower(1/LowerPowerBy);
            }
            else if (ButtonMonitor.isPressed(buttonName.dpad_down)){
                actuatorDrive.setPower(-1/LowerPowerBy);
            }

            telemetry.addData("Current Pos", SlideTurniCurrentPos);
            telemetry.addData("Current Angle", SlideTurniAngle);
            telemetry.addData("Current Power", slideyTurni.getPower());
            telemetry.addData("Dir", slideyTurni.getDirection());
            telemetry.addData("Slidey Pos", (linearSlidey).getCurrentPosition());
            telemetry.update();
            }
        }

    private int slideyAngleToPos(double angle){
        final double GearRatio4 = 3.61;
        final double GearRatio5 = 5.23;
        final double DriveHDHexMotorCPR = 28 * GearRatio5 * GearRatio4;
        return (int) (angle/360*DriveHDHexMotorCPR);
    }

    private double slideyPostoAngle(int Pos){
        final double GearRatio4 = 3.61;
        final double GearRatio5 = 5.23;
        final double DriveHDHexMotorCPR = 28 * GearRatio5 * GearRatio4;
        return Pos / DriveHDHexMotorCPR * 360;
    }

    private void setZPFloat(boolean isGamepad2) {
        if (isGamepad2) {
            linearSlidey.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            actuatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            slideyTurni.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }
    private void setZPBrake(boolean isGamepad2) {
        if (isGamepad2) {
            linearSlidey.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            actuatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            slideyTurni.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    // This button status thing is inspired by u/m0stlyharmless_user and u/fusionforscience on reddit from a post 8y ago :)
    // https://www.reddit.com/r/FTC/comments/5lpaai/comment/dbye175/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button
    // https://www.reddit.com/r/FTC/comments/5lpaai/comment/dcerspj/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button
    enum Status {
        notPressedYet,
        currentlyPressed,
        wasPressed
    }
    enum buttonName {
        options,
        triangle,
        share,
        cross,
        square,
        circle,
        left_stick_button,
        right_stick_button,
        dpad_left,
        dpad_right,
        dpad_up,
        dpad_down,
        right_bumper,
        left_bumper
    }
    enum Gpads { //GP stands for gamepad
        GP1,
        GP2
    }
    private class Buttons {
        public  final EnumMap<buttonName, Status> buttonMap     = new EnumMap<buttonName, Status>(buttonName.class);
        private final EnumMap<buttonName, Button> ButtonStorage = new EnumMap<buttonName, Button>(buttonName.class);
        private Gpads GP;
        private Gamepad gpad = new Gamepad();
        private boolean[] buttonList;

        public Buttons(boolean isGamepad2) {
            GP = Gpads.GP1;
            if (isGamepad2) {
                GP = Gpads.GP2;
            }
            for (buttonName button : buttonName.values()) {
                ButtonStorage.put(button, new Button());
            }
        }

        public boolean Pressed(buttonName button) {
            return buttonMap.get(button) == Status.wasPressed || buttonMap.get(button) == Status.currentlyPressed;
        }

        public boolean wasPressed(buttonName button) {
            return buttonMap.get(button) == Status.wasPressed;
        }

        public boolean isPressed(buttonName button) {
            return buttonMap.get(button) == Status.currentlyPressed;
        }

        public boolean NotPressed(buttonName button){
            return buttonMap.get(button) == Status.notPressedYet;
        }

        public void update(){
            if (GP == Gpads.GP1) {
                gpad.copy(gamepad1);
            }
            else {
                gpad.copy(gamepad2);
            }
            buttonList = new boolean[] {gpad.options, gpad.triangle, gpad.share, gpad.cross, gpad.square, gpad.circle, gpad.left_stick_button, gpad.right_stick_button, gpad.dpad_left, gpad.dpad_right, gpad.dpad_up, gpad.dpad_down, gpad.right_bumper, gpad.left_bumper};
            buttonName[] ButtonArr = buttonName.values();
            for (int i=0; i<buttonList.length; i++) {
                buttonMap.put(ButtonArr[i], ButtonStorage.get(ButtonArr[i]).ButtonStatus(buttonList[i]));
            }
        }

        private class Button { //class in a class in a class for funsies
            private Status status = Status.notPressedYet;
            public Status ButtonStatus(boolean button) {
                if ( (button && status == Status.notPressedYet))
                    status = Status.currentlyPressed;
                else if (!button && status == Status.currentlyPressed)
                    status = Status.wasPressed;
                else if (status == Status.wasPressed)
                    status = Status.notPressedYet;
                return status;
            }
        }
    }
}