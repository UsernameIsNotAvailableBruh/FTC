package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.EnumMap;


@TeleOp(name="FancyNoTouchyOpDickstein", group="OpMode")
public class FancyOpDickstein extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor actuator1 = null;
    private DcMotor actuator2 = null;

    private DcMotor pitch = null;

    private CRServo spinServo = null;

    private Servo blockServo = null;
    private boolean isBlocking = false;

    private static boolean GP2Initialized = false;

    @Override
    public void runOpMode() {
        leftBackDrive  = hardwareMap.get(DcMotor.class, "lb"); // 0
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "lf"); // 1
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb"); // 2
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf"); // 3

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotorSimple.Direction.FORWARD);

        actuator1 = hardwareMap.get(DcMotor.class, "actuator1");
        actuator2 = hardwareMap.get(DcMotor.class, "actuator2");
        actuator1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        actuator2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pitch = hardwareMap.get(DcMotor.class, "pitch");
        pitch.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pitch.setDirection(DcMotor.Direction.REVERSE);

        spinServo = hardwareMap.get(CRServo.class, "spinServo");
        blockServo = hardwareMap.get(Servo.class, "blockServo");

        // Wait for the game to start (driver presses PLAY)
        while (!GP2Initialized){
            sleep(100);
        }
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Thread gamepad2Thread = new Thread( new Gamepad2Thread() );
        gamepad2Thread.setDaemon(false);
        gamepad2Thread.start();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // Normal controls:
            double axial = Math.pow(-gamepad1.left_stick_y, 3);  // Note: pushing stick forward gives negative value
            double lateral = Math.pow(gamepad1.left_stick_x, 3);
            double yaw = Math.pow(gamepad1.right_trigger - gamepad1.left_trigger, 3);

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            double Slow = 1;
            if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
                Slow = 2;
            }

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower / Slow);
            rightFrontDrive.setPower(rightFrontPower / Slow);
            leftBackDrive.setPower(leftBackPower / Slow);
            rightBackDrive.setPower(rightBackPower / Slow);


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("isBlocking", isBlocking);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Actuator1", "pos: %d, power: %4.2f", actuator1.getCurrentPosition(), actuator1.getPower());
            telemetry.addData("Actuator2", "pos: %d, power: %4.2f", actuator2.getCurrentPosition(), actuator2.getPower());
            telemetry.addData("Pitch", "pos: %d, power: %4.2f", pitch.getCurrentPosition(), pitch.getPower());
            telemetry.addData("Spin Servo", "power: %4.2f", spinServo.getPower());
            telemetry.addData("Block servo", "pos: %4.2f", blockServo.getPosition());
            telemetry.update();
        }
    }

    public class Gamepad2Thread implements Runnable {
        public void run() {
            Buttons ButtonMonitor = new Buttons();
            GP2Initialized = true;
            telemetry.addData("GP2 Status", "Initialized");
            waitForStart();

            runtime.reset();
            while (opModeIsActive()) {
                ButtonMonitor.update();

                double actuator1Power = Math.pow(-gamepad2.left_stick_y, 3);
                double actuator2Power = Math.pow(-gamepad2.right_stick_y, 3);
                double deadzone = 0.35;
                actuator1Power = (Math.abs(actuator1Power) < deadzone) ? 0 : actuator1Power;
                actuator2Power = (Math.abs(actuator2Power) < deadzone) ? 0 : actuator2Power;
                double slowDownFactor = 1;
                actuator1.setPower(actuator1Power / slowDownFactor);
                actuator2.setPower(actuator2Power / slowDownFactor);

                double pitchPower = Math.pow(gamepad2.right_trigger - gamepad2.left_trigger, 3);
                double pitchSlowDownFactor = 1.5;
                if (pitchPower > 0 && pitch.getCurrentPosition() < 2300) {
                    pitch.setPower(pitchPower / pitchSlowDownFactor);
                } else if (pitchPower < 0 && pitch.getCurrentPosition() > -50) {
                    pitch.setPower(pitchPower / pitchSlowDownFactor);
                } else {
                    pitch.setPower(0);
                }

                if (ButtonMonitor.wasPressed(buttonName.a)) {
                    spinServo.setPower(1);
                } else if (ButtonMonitor.wasPressed(buttonName.y)) {
                    spinServo.setPower(-1);
                } else {
                    spinServo.setPower(0);
                }

                if (ButtonMonitor.wasPressed(buttonName.x)) {
                    isBlocking = !isBlocking;
                }
                if (isBlocking) {
                    blockServo.setPosition(0);
                } else if (!isBlocking) {
                    blockServo.setPosition(.7); //.8
                }
            }
        }
    }

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
        left_bumper,
        a,
        y,
        x
    }
    private class Buttons {
        public  final EnumMap<buttonName, Status> buttonMap = new EnumMap<>(buttonName.class);
        private final EnumMap<buttonName, Buttons.Button> ButtonStorage = new EnumMap<>(buttonName.class);
        private Gamepad gpad = new Gamepad();
        private boolean[] buttonList;

        public Buttons() {
            for (buttonName button : buttonName.values()) {
                ButtonStorage.put(button, new Buttons.Button());
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
            gpad.copy(gamepad2);
            buttonList = new boolean[] {gpad.options, gpad.triangle, gpad.share, gpad.cross, gpad.square, gpad.circle, gpad.left_stick_button, gpad.right_stick_button, gpad.dpad_left, gpad.dpad_right, gpad.dpad_up, gpad.dpad_down, gpad.right_bumper, gpad.left_bumper, gpad.a, gpad.y, gpad.x};
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
