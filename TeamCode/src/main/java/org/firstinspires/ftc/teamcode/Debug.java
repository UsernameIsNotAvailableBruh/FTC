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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Gamepad.LedEffect;
import com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect;
import com.qualcomm.robotcore.util.ReadWriteFile;

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

//BHI260AP

@TeleOp(name="Debuggy", group="OpMode")
public class Debug extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo LeftServo = null;
    private Servo RightServo = null;
    private Servo TurnServo = null;
    private DcMotor leftFrontDrive = null;

    @Override
    public void runOpMode() {
        //https://gm0.org/en/latest/docs/software/tutorials/gamepad.html#storing-gamepad-state
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        //  LeftServo = hardwareMap.get(Servo.class, "serv1");
        //  RightServo = hardwareMap.get(Servo.class, "serv2");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        //TurnServo = hardwareMap.get(Servo.class, "turn");

        //rumble (just for funsies)
        RumbleEffect.Builder rumble = new RumbleEffect.Builder();
        for (double i = 0; i < 1; i += .1) {
            rumble = rumble.addStep(i, 1 - i, 100);
            rumble = rumble.addStep(1 - i, i, 100);
        }
        gamepad1.runRumbleEffect(rumble.build());
        gamepad2.runRumbleEffect(rumble.build());

        //LED effects (also for funsies)
        LedEffect.Builder Led = new LedEffect.Builder();
        int DurationMs = 20;
        double AddValue = .1;
        // R to G
        for (double i = 0; i <= 1; i += AddValue) { //R to Black
            double RoundedI = (Math.round(i * 100) / 100.0); //turns something like 3.00000004 into 3.0
            Led = Led.addStep(1 - RoundedI, 0, 0, DurationMs);
        }
        for (double i = 0; i <= 1; i += AddValue) { //Black to G
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(0, RoundedI, 0, DurationMs);
        }
        // G to B
        for (double i = 0; i <= 1; i += AddValue) { //G to Black
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(0, 1 - RoundedI, 0, DurationMs);
        }
        for (double i = 0; i <= 1; i += AddValue) { //Black to B
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(0, 0, RoundedI, DurationMs);
        }
        // B to R
        for (double i = 0; i <= 1; i += AddValue) { //B to Black
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(0, 0, 1 - RoundedI, DurationMs);
        }
        for (double i = 0; i <= 1; i += AddValue) { //Black to R
            double RoundedI = (Math.round(i * 100) / 100.0);
            Led = Led.addStep(RoundedI, 0, 0, DurationMs);
        }
        Led.setRepeating(true);
        gamepad1.runLedEffect(Led.build());
        gamepad2.runLedEffect(Led.build());
        // LeftServo.setDirection(Servo.Direction.REVERSE);
        // Wait for the game to start (driver presses START)
        // LeftServo.setPosition(0);
        // RightServo.setPosition(0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();
        // boolean ClawToggle = false;

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setPower(1);
        sleep(1000);
        leftFrontDrive.setPower(0);
        int a = leftFrontDrive.getCurrentPosition();

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setPower(.5);
        sleep(1000);
        leftFrontDrive.setPower(0);
        int b = leftFrontDrive.getCurrentPosition();
        ReadWriteFile.writeFile(ROBOT_DATA_DIR, "PosPowerData.txt", String.valueOf(a)+ " "+String.valueOf(b));


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setPower(-1);
        sleep(1000);
        leftFrontDrive.setPower(0);
        a = leftFrontDrive.getCurrentPosition();

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setPower(-.5);
        sleep(1000);
        leftFrontDrive.setPower(0);
        b = leftFrontDrive.getCurrentPosition();
        ReadWriteFile.writeFile(ROBOT_DATA_DIR, "PosPowerReverseData.txt", String.valueOf(a)+ " "+String.valueOf(b));

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Pos", "%4.2f", leftFrontDrive.getCurrentPosition());
        telemetry.update();
    }
}