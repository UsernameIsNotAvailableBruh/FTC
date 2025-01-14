package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode; lower level version of LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

// *--*
// All the code is written by Aajinkya Naik unless stated otherwise.
// *--*

@Autonomous(name="AutoDickstein", group = "OpDicksteinModes")
public class OmniAutoMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private IMU BHI260AP = null;

    final double GearRatio3 =  2.89;
    final double GearRatio4 = 3.61;
    final double GearRatio5 = 5.23;
    final double DriveHDHexMotorCPR = 28 * GearRatio5 * GearRatio4;
    final double goBILDAWheel = 3.77953; //96 mm -> in \\diameter
    final double WheelCircum = Math.PI * goBILDAWheel;
    double leftFrontPower  = 1;
    double rightBackPower  = 1;
    double leftBackPower   = 1;
    double rightFrontPower = 1;
    @Override
    public void runOpMode() {
        waitForStart();
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront"); //Motor 0 = left front
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront"); //Motor 1 = right front
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack"); //Motor 2 = right bottom
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack"); //Motor 3 = left bottom
        BHI260AP = hardwareMap.get(IMU.class, "imu");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        IMU.Parameters IMUParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );
        BHI260AP.initialize(IMUParams);
        BHI260AP.resetYaw();
        ResetEncoders();

        int leftFrontEncoderPos = leftFrontDrive.getCurrentPosition();
        int rightFrontEncoderPos = rightFrontDrive.getCurrentPosition();
        int rightBackEncoderPos = rightBackDrive.getCurrentPosition();
        int leftBackEncoderPos = leftBackDrive.getCurrentPosition();

        double LFRevolutions = leftFrontEncoderPos/DriveHDHexMotorCPR;
        double LBRevolutions = leftBackEncoderPos/DriveHDHexMotorCPR;
        double RFRevolutions = rightFrontEncoderPos/DriveHDHexMotorCPR;
        double RBRevolutions = rightBackEncoderPos/DriveHDHexMotorCPR;

        double LFAngle = LFRevolutions*360%360;
        double LBAngle = LBRevolutions*360%360;
        double RFAngle = RFRevolutions*360%360;
        double RBAngle = RBRevolutions*360%360;

        double LFDistance = WheelCircum*LFRevolutions;
        double LBDistance = WheelCircum*LBRevolutions;
        double RFDistance = WheelCircum*RFRevolutions;
        double RBDistance = WheelCircum*RBRevolutions;

        leftFrontPower  *= .5;
        rightBackPower  *= .5;
        leftBackPower   *= -.5;
        rightFrontPower *= -.5;
        leftFrontDrive.setPower(leftFrontPower);
        rightBackDrive.setPower(rightBackPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        sleep(1000);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addLine("Encoder");
        telemetry.addData("LF/RF Encoder", "%d, %d", leftFrontEncoderPos, rightFrontEncoderPos);
        telemetry.addData("LB/RB Encoder", "%d, %d", leftBackEncoderPos, rightBackEncoderPos);
        telemetry.addLine("Revolution");
        telemetry.addData("LF/RF Revolutions", "%4.2f, %4.2f",LFRevolutions, RFRevolutions);
        telemetry.addData("LB/RB Revolutions", "%4.2f, %4.2f",LBRevolutions, RBRevolutions);
        telemetry.addLine("Angle");
        telemetry.addData("LF/RF Angle", "%4.2f, %4.2f", LFAngle, RFAngle);
        telemetry.addData("LB/RB Angle", "%4.2f, %4.2f", LBAngle, RBAngle);
        telemetry.addLine("Distance");
        telemetry.addData("LF/RF Distance", "%4.2f, %4.2f", LFDistance, RFDistance);
        telemetry.addData("LB/RB Distance", "%4.2f, %4.2f", LBDistance, RBDistance);

        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    public void ResetEncoders() {
        // Reset the motor encoder so that it reads zero ticks
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Turn the motor back on, required if you use STOP_AND_RESET_ENCODER
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    enum Wheel {
        LF,
        RF,
        LB,
        RB,
        }
    public void GotoPositionForward(int Pos) {
        leftFrontDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPowerALL(1);
    }

    public void setPowerALL(double Power) {
        rightFrontDrive.setPower(Power);
        rightBackDrive.setPower(Power);
        leftFrontDrive.setPower(Power);
        leftBackDrive.setPower(Power);
    }

    public int InchtoPos(double Inches) {
        double CountsPerInch = 2*DriveHDHexMotorCPR / WheelCircum;
        return (int) (Inches*CountsPerInch);
    }

    public void GotoAtAngle(double theta, double Power, int Pos) {
        double YawOffset = BHI260AP.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).thirdAngle;

        double Direction1 = Math.sin(theta + Math.PI/4 - YawOffset); // https://www.desmos.com/calculator/rqqamhfeek
        double Direction2 = Math.sin(theta - Math.PI/4 - YawOffset); // https://www.desmos.com/calculator/dminewe5vs

        double leftFrontPower  = Direction1 * Power;
        double rightBackPower  = Direction1 * Power;
        double leftBackPower   = Direction2 * Power;
        double rightFrontPower = Direction2 * Power;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        leftFrontDrive.setTargetPosition((leftFrontDrive.getCurrentPosition()+Pos)*1915); // Tells the motor that the position it should go to is desiredPosition
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(leftFrontPower);

        rightFrontDrive.setTargetPosition((rightFrontDrive.getCurrentPosition()+Pos)*1915); // Tells the motor that the position it should go to is desiredPosition
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(rightFrontPower);

        rightBackDrive.setTargetPosition((rightBackDrive.getCurrentPosition()+Pos)*1915); // Tells the motor that the position it should go to is desiredPosition
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(rightBackPower);

        leftBackDrive.setTargetPosition((leftBackDrive.getCurrentPosition()+Pos)*1915); // Tells the motor that the position it should go to is desiredPosition
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(leftBackPower);
    }
}