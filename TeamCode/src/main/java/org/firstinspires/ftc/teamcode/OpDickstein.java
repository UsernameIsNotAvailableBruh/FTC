package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="OpDickstein", group="OpMode")
public class OpDickstein extends LinearOpMode {

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
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

            if (gamepad2.a) {
                spinServo.setPower(1);
            } else if (gamepad2.y) {
                spinServo.setPower(-1);
            } else {
                spinServo.setPower(0);
            }

            if (gamepad2.x) {
                isBlocking = !isBlocking;
            }
            if (isBlocking) {
                blockServo.setPosition(.55); //.35
            } else if (!isBlocking) {
                blockServo.setPosition(.5);
            }

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
}
