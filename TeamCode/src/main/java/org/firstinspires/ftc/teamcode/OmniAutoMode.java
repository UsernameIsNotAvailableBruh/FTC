package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode; lower level version of LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoDickstein", group = "OpDicksteinModes")
public class OmniAutoMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront"); //Motor 0 = left front
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront"); //Motor 1 = right front
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack"); //Motor 2 = right bottom
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack"); //Motor 3 = left bottom

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        ResetEncoders();

        int leftFrontEncoderPos = leftFrontDrive.getCurrentPosition();
        int rightFrontEncoderPos = rightFrontDrive.getCurrentPosition();
        int rightBackEncoderPos = rightBackDrive.getCurrentPosition();
        int leftBackEncoderPos = leftBackDrive.getCurrentPosition();

        double GearRatio3 =  2.89;
        double GearRatio4 = 3.61;
        double GearRatio5 = 5.23;
        double DriveHDHexMotorCPR = 28 * GearRatio5 * GearRatio4;

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("leftFrontPos", "%4.2d", leftFrontPos);
        telemetry.addData("rightFrontPos", "%4.2d", rightFrontPos);
        telemetry.addData("rightBackPos", "%4.2d", rightBackPos);
        telemetry.addData("leftBackPos", "%4.2d", leftBackPos);

        telemetry.addData("Encoder Front left/Right", "%4.2f, %4.2f", leftFrontEncoderPos, rightFrontEncoderPos);
        telemetry.addData("Encoder Back  left/Right", "%4.2f, %4.2f", leftBackEncoderPos, rightBackEncoderPos);

        telemetry.update();
    }
    public void ResetEncoders(){
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
}