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




import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.Gamepad.LedEffect;
import com.qualcomm.robotcore.hardware.Gamepad.RumbleEffect;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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

@TeleOp(name="OpDickstein", group="OpMode")
public class OmniOpMode extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime     = new ElapsedTime();
    private DcMotor leftFrontDrive  = null;
    private DcMotor leftBackDrive   = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive  = null;
    private DcMotor SlideyDrive     = null;
    private IMU BHI260AP            = null;
    private Servo LeftServo         = null;
    private Servo RightServo        = null;
    private Servo TurnServo         = null;
    private DcMotor ActuatorDrive   = null;

    @Override
    public void runOpMode() {
        //https://gm0.org/en/latest/docs/software/tutorials/gamepad.html#storing-gamepad-state
        Gamepad currentGamepad1  = new Gamepad();
        Gamepad currentGamepad2  = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();
        Gamepad previousGamepad2 = new Gamepad();

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront"); //Motor 0 = left front
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront"); //Motor 1 = right front
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBack"); //Motor 2 = right bottom
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBack"); //Motor 3 = left bottom

        SlideyDrive = hardwareMap.get(DcMotor.class, "slidey");
        ActuatorDrive = hardwareMap.get(DcMotor.class, "actorio");

        LeftServo = hardwareMap.get(Servo.class, "serv1"); //port 0 - the one closer to the linear slider
        RightServo = hardwareMap.get(Servo.class, "serv2"); //port 1
        TurnServo = hardwareMap.get(Servo.class, "serv3"); //port 2

        BHI260AP = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters IMUParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        BHI260AP.initialize(IMUParams);
        BHI260AP.resetYaw();

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        SlideyDrive.setDirection(DcMotor.Direction.REVERSE);
        ActuatorDrive.setDirection(DcMotor.Direction.REVERSE);


        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideyDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SlideyDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double GearRatio3 =  2.89;
        double GearRatio4 = 3.61;
        double GearRatio5 = 5.23;
        double DriveHDHexMotorCPR = 28 * GearRatio5 * GearRatio4;

        //rumble (just for funsies)
        RumbleEffect.Builder rumble = new RumbleEffect.Builder();
        for (double i=0;i<1;i+=.1){
            rumble = rumble.addStep(i, 1-i, 100);
            rumble = rumble.addStep(1-i, i, 100);
        }
        //gamepad1.runRumbleEffect(rumble.build()); //hopefully this works, idk
        //gamepad2.runRumbleEffect(rumble.build());

        //LED effects (also for funsies)
        int DurationMs = 10;
        double AddValue = .01;
        Effects effects = new Effects(AddValue, DurationMs);
        LedEffect.Builder Led = effects.RGB;
        Led.setRepeating(true);
        gamepad1.runLedEffect(Led.build());
        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        boolean ZPBehaviorToggle = true; //True is float
        boolean ClawToggle = true;
        boolean ClawToggle2 = true;
        double YawOffset = 0;
        RightServo.setDirection(Servo.Direction.REVERSE);
        LeftServo.setPosition(0);
        RightServo.setPosition(0);
        TurnServo.setPosition(.1);
        BHI260AP.resetYaw();
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1); //gamepad from last iteration
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            Orientation robotOrientation = BHI260AP.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
            );

            int leftFrontDriveEncoderPos  = leftFrontDrive.getCurrentPosition();
            int rightFrontDriveEncoderPos = rightFrontDrive.getCurrentPosition();
            int rightBackDriveEncoderPos  = rightBackDrive.getCurrentPosition();
            int leftBackDriveEncoderPos   = leftBackDrive.getCurrentPosition();

            // Rising Edge Detector for (gamepad1.left_stick_button && gamepad1.right_stick_button)
            if (gamepad1.options && !previousGamepad1.options) {
                ZPBehaviorToggle = !ZPBehaviorToggle;
            }
            if (ZPBehaviorToggle)
                setZPFloat();
            else
                setZPBrake();


            //claw stuff
            //SlideyDrive.setPower(-gamepad2.left_stick_y);
            //ActuatorDrive.setPower(-gamepad2.right_stick_y);
            if (gamepad2.cross && !previousGamepad2.cross) {
                ClawToggle = !ClawToggle; //if its true make it false, if its false make it true
            }
            if (ClawToggle) {
                LeftServo.setPosition(.5);
                RightServo.setPosition(.5);
            }
            else if (!ClawToggle) {
                LeftServo.setPosition(0);
                RightServo.setPosition(0);
            }

            if (gamepad2.circle && !previousGamepad2.circle) {
                ClawToggle2 = !ClawToggle2;
            }
            if (ClawToggle2) {
                TurnServo.setPosition(.1);
            }
            else if (!ClawToggle2) {
                TurnServo.setPosition(.55);
            }

            if (gamepad2.left_bumper){
                TurnServo.setPosition(gamepad2.left_trigger);
            }
            // Rising Edge Detector to dance
            // while ((gamepad1.left_stick_button && gamepad1.right_stick_button) && !(previousGamepad1.left_stick_button && previousGamepad1.right_stick_button)) {
            //     happyDanceRobot();
            // }

            double max;
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            /*
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;
            */
            /*
            1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
            2) Lateral:  Strafing right and left                     Left-joystick Right and Left
            3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
            */

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            /*double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;*/

            // hypotenuse = power
            // slope = direction
            double lefty = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double leftx = gamepad1.left_stick_x;
            double hypotenuse = Math.sqrt(  Math.pow(leftx, 2)+Math.pow(lefty, 2)  ); //pythagorean theorem

            double Slidey = -gamepad2.right_stick_y;
            double Acturio = -gamepad2.left_stick_y;

            SlideyDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            SlideyDrive.setPower(Slidey);

            ActuatorDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ActuatorDrive.setPower(Acturio);

            /*
            slope is basically the direction the robot is gonna go
            hypotenuse is power

            slope * hypotenuse is power for each wheel (hopefully)
            */

            double leftFrontPower  = hypotenuse;
            double rightFrontPower = hypotenuse;
            double leftBackPower   = hypotenuse;
            double rightBackPower  = hypotenuse;

            //https://gm0.org/en/latest/_images/mecanum-drive-directions.png
            //https://cdn11.bigcommerce.com/s-x56mtydx1w/images/stencil/original/products/1445/7196/3213-3606-0002-Product-Insight-3__67245__45972.1701993091.png?c=1

            //https://youtu.be/gnSW2QpkGXQ?si=lnVXFP7B3FyuYVPt - this video is EXTREMELY helpful
            //https://seamonsters-2605.github.io/archive/mecanum/ - this website i found from reddit is helpful too

            double theta = Math.atan2(lefty, leftx);
            double Roll  = robotOrientation.firstAngle; // X - Roll
            double Pitch = robotOrientation.secondAngle; // Y - Pitch
            double Yaw   = robotOrientation.thirdAngle; // Z - Yaw

            if (gamepad1.cross && !previousGamepad1.cross)
                YawOffset = resetYaw();
            if (gamepad1.square && !previousGamepad1.square)
                BHI260AP.resetYaw();

            telemetry.addData("Theta value\t", "%4.2f", theta);
            telemetry.addData("X - Roll\t", "%4.2f", Roll);
            telemetry.addData("Y - Pitch\t", "%4.2f", Pitch);
            telemetry.addData("Z - Yaw\t", "%4.2f", Yaw);

            double Direction1 = Math.sin(theta + Math.PI/4 - YawOffset); // https://www.desmos.com/calculator/rqqamhfeek
            double Direction2 = Math.sin(theta - Math.PI/4 - YawOffset); // https://www.desmos.com/calculator/dminewe5vs

            leftFrontPower  *= Direction1;
            rightBackPower  *= Direction1;
            leftBackPower   *= Direction2;
            rightFrontPower *= Direction2;

            // TODO: add dpad stuff again, but with gyroscope stuff
            //if (gamepad1.dpad_up) {
            //    //sin(Math.PI/2) is 1
            //    double RAD = Math.PI/2;
            //    double Dpadirection1 = Math.sin(RAD + Math.PI/4 - YawOffset);
            //    double Dpadirection2 = Math.sin(RAD - Math.PI/4 - YawOffset);
            //    // Dpadirection1 is Dpadirection2
            //    leftFrontPower  = Dpadirection1 * hypotenuse; // 1
            //    rightFrontPower = Dpadirection2 * hypotenuse; // 1
            //    leftBackPower   = Dpadirection2 * hypotenuse; // 1
            //    rightBackPower  = Dpadirection1 * hypotenuse; // 1
            //} else if (gamepad1.dpad_down) {
            //    double RAD = Math.PI*3.0/2;
            //    double Dpadirection1 = Math.sin(RAD + Math.PI/4 - YawOffset);
            //    double Dpadirection2 = Math.sin(RAD - Math.PI/4 - YawOffset);
            //    // Dpadirection1 is Dpadirection2
            //    leftFrontPower  = Dpadirection1 * hypotenuse; // -1
            //    rightFrontPower = Dpadirection2 * hypotenuse; // -1
            //    leftBackPower   = Dpadirection2 * hypotenuse; // -1
            //    rightBackPower  = Dpadirection1 * hypotenuse; // -1
            //} else if (gamepad1.dpad_left) {
            //    double RAD = Math.PI*3.0/2;
            //    double RAD2 = Math.PI/2;
            //    double Dpadirection1 = Math.sin(RAD + Math.PI/4 - YawOffset);  // -1
            //    double Dpadirection2 = Math.sin(RAD2 - Math.PI/4 - YawOffset); //  1
            //    leftFrontPower  = Dpadirection1 * hypotenuse; // -1
            //    rightFrontPower = Dpadirection2 * hypotenuse; //  1
            //    leftBackPower   = Dpadirection2 * hypotenuse; //  1
            //    rightBackPower  = Dpadirection1 * hypotenuse; // -1
            //} else if (gamepad1.dpad_right) {
            //    double RAD = Math.PI/2;
            //    double RAD2 = Math.PI*3.0/2;
            //    double Dpadirection1 = Math.sin(RAD + Math.PI/4 - YawOffset);  // -1
            //    double Dpadirection2 = Math.sin(RAD2 - Math.PI/4 - YawOffset); //  1
            //    leftFrontPower  = Dpadirection1 * hypotenuse; //  1
            //    rightFrontPower = Dpadirection2 * hypotenuse; // -1
            //    leftBackPower   = Dpadirection2 * hypotenuse; // -1
            //    rightBackPower  = Dpadirection1 * hypotenuse; //  1
            //}


            leftFrontPower -= gamepad1.left_trigger;
            rightFrontPower += gamepad1.left_trigger;
            leftBackPower -= gamepad1.left_trigger;
            rightBackPower += gamepad1.left_trigger;

            leftFrontPower += gamepad1.right_trigger;
            rightFrontPower -= gamepad1.right_trigger;
            leftBackPower += gamepad1.right_trigger;
            rightBackPower -= gamepad1.right_trigger;
            YawOffset = resetYaw();

            if (gamepad1.right_bumper) {
                leftFrontPower  = 1;
                rightFrontPower = -1;
                leftBackPower   = 1;
                rightBackPower  = -1;
                YawOffset = resetYaw();
            } else if (gamepad1.left_bumper) {
                leftFrontPower  = -1;
                rightFrontPower = 1;
                leftBackPower   = -1;
                rightBackPower  = 1;
                YawOffset = resetYaw();
            }

            if (gamepad1.left_stick_button) //make the other controller rumble
                gamepad2.rumble(100);
            if (gamepad2.left_stick_button)
                gamepad1.rumble(100);

            if (gamepad1.right_stick_button) //stop rumbles
                gamepad1.stopRumble();
            if (gamepad2.right_stick_button)
                gamepad2.stopRumble();

            // Normalize the values so no wheel  power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }


            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Mem", "Run Time: " + Runtime.getRuntime().totalMemory());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Left, Right, Turn", "%4.2f, %4.2f, %4.2f", LeftServo.getPosition(), RightServo.getPosition(), TurnServo.getPosition());
            telemetry.update();
        }
    }
    private void happyDanceRobot() {
        setZPFloat();
        leftFrontDrive.setPower(1);
        rightFrontDrive.setPower(-1);
        leftBackDrive.setPower(1);
        rightBackDrive.setPower(-1);
        sleep(1500);
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        sleep(1000);
        leftFrontDrive.setPower(-1);
        rightFrontDrive.setPower(1);
        leftBackDrive.setPower(-1);
        rightBackDrive.setPower(1);
        sleep(1500);
    }
    private void setZPFloat() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        SlideyDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    private void setZPBrake() {
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        SlideyDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private double resetYaw() {
        double Yaw = BHI260AP.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
            ).thirdAngle;
        return Yaw;
    }
}