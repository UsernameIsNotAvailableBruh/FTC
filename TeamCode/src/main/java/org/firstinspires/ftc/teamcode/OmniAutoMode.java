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

        double goBILDAWheel = 3.77953; //96 mm -> in
        double WheelCircum = Math.PI * goBILDAWheel;

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

        leftFrontDrive.setTargetPosition(1000); // Tells the motor that the position it should go to is desiredPosition
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(0.5);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addLine("Encoder");
        telemetry.addData("LF/RF Encoder", "%4.2f, %4.2f", leftFrontEncoderPos, rightFrontEncoderPos);
        telemetry.addData("LB/RB Encoder", "%4.2f, %4.2f", leftBackEncoderPos, rightBackEncoderPos);
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