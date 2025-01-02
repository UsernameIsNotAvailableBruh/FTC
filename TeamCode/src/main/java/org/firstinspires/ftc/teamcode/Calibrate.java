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

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.util.ReadWriteFile;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

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

@TeleOp(name="Calibrate")
public class Calibrate extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo LeftServo = null;
    private Servo RightServo = null;
    private Servo TurnServo = null;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor rightBackDrive;
    private DcMotor leftBackDrive;
    ArrayList<ArrayList<Integer>> Data = new ArrayList<ArrayList<Integer>>();

    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        //  LeftServo = hardwareMap.get(Servo.class, "serv1");
        //  RightServo = hardwareMap.get(Servo.class, "serv2");
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBack");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBack");
        //TurnServo = hardwareMap.get(Servo.class, "turn");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();
        // boolean ClawToggle = false;

        double[][] PowernSecLevels = new double[][] {{1,  0}, {-1,  0}, {.5,  0}, {-.5,  0}, {.1,  0}, {-.1,  0}, {0,  0},
                                                     {1,.25}, {-1,.25}, {.5,.25}, {-.5,.25}, {.1,.25}, {-.1,.25}, {0,.25}};
                                                     //{1, .5}, {-1, .5}, {.5, .5}, {-.5, .5}, {.1, .5}, {-.1, .5}, {0, .5},
                                                     //{1,.75}, {-1,.75}, {.5,.75}, {-.5,.75}, {.1,.75}, {-.1,.75}, {0,.75},
                                                     //{1,  1}, {-1,  1}, {.5,  1}, {-.5,  1}, {.1,  1}, {-.1,  1}, {0,  1}};

        Data.add(0, new ArrayList<Integer>()); // leftFrontDrive
        Data.add(1, new ArrayList<Integer>()); // rightFrontDrive
        Data.add(2, new ArrayList<Integer>()); // leftBackDrive
        Data.add(3, new ArrayList<Integer>()); // rightBackDrive

        for (double[] PowernSecLevel : PowernSecLevels) {
            int i = 0;
            for (int Pos : PowerTest(PowernSecLevel[0], PowernSecLevel[1])) {
                Data.get(i).add(Pos);
                i++;
            }
        }

        //TODO get linear regression slope

        MultipleLinearRegression EquationLF = new MultipleLinearRegression(PowernSecLevels, YPointFinder(0));
        MultipleLinearRegression EquationRF = new MultipleLinearRegression(PowernSecLevels, YPointFinder(1));
        MultipleLinearRegression EquationLB = new MultipleLinearRegression(PowernSecLevels, YPointFinder(2));
        MultipleLinearRegression EquationRB = new MultipleLinearRegression(PowernSecLevels, YPointFinder(3));

        /*String fileContents = String.valueOf(m)+ " "+String.valueOf(b) + "\n";
        b = EquationRF.intercept();
        m = EquationRF.slope();
        fileContents += String.valueOf(m)+ " "+String.valueOf(b) + "\n";
        b = EquationLB.intercept();
        m = EquationLB.slope();
        fileContents += String.valueOf(m)+ " "+String.valueOf(b) + "\n";
        b = EquationRB.intercept();
        m = EquationRB.slope();
        fileContents += String.valueOf(m)+ " "+String.valueOf(b);
        ReadWriteFile.writeFile(ROBOT_DATA_DIR, "PosPowerEquations.txt", fileContents);*/

        String fileContents = String.valueOf(EquationLF.beta(0))+" "+String.valueOf(EquationLF.beta(1));
        FileWriter Fw;
        try {
            Fw = new FileWriter(new File(ROBOT_DATA_DIR, "PosPowerEquations.txt"));
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        try {
            Fw.write(fileContents);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        try {
            Fw.flush();
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        // Show the elapsed game time and wheel power.
        PowerTest(.1, 5);
        telemetry.update();
        sleep(10_000);
        telemetry.update();
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        //telemetry.addData("Pos", "%d", Drive.getCurrentPosition());
        telemetry.update();
    }

    public int[] PowerTest(double power, double sec) {
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setPower(power);
        rightFrontDrive.setPower(power);
        leftBackDrive.setPower(power);
        rightBackDrive.setPower(power);
        sleep((int) (sec*1000));
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        return new int[] {leftFrontDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition()};
    }
    private double[] YPointFinder(int MotorInt) {
        double[] YPoints = new double[Data.get(MotorInt).size()];
        int i = 0;
        for (Object ele: Data.get(MotorInt).toArray()){
            YPoints[i] = (int)Double.parseDouble(ele.toString());
            i++;
        }
        return YPoints;
    }
}