package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode; lower level version of LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="AutoDickstein", group = "OpDicksteinModes")
public class OmniAutoMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private IMU BHI260AP = null;


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

        double CountsPerInch = DriveHDHexMotorCPR / WheelCircum;

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

        GotoPositionForward(1000); // Tells the motor that the position it should go to is desiredPosition


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

    public void GotoPositionForward(int Pos) {
        leftFrontDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPowerALL(0.5);
    }

    public void setPowerALL(double Power) {
        rightFrontDrive.setPower(Power);
        rightBackDrive.setPower(Power);
        leftFrontDrive.setPower(Power);
        leftBackDrive.setPower(Power);
    }

    public void GotoPositionForwardInches(int Pos, double Power) {
        leftFrontDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFrontDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightBackDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftBackDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        setPowerALL(Power);
    }

    public void Angle(int theta, double Power) {
        double YawOffset = BHI260AP.getRobotOrientation(
                AxesReference.INTRINSIC,
                AxesOrder.XYZ,
                AngleUnit.RADIANS
        ).thirdAngle;

        double Direction1 = Math.sin(theta + Math.PI/4 - YawOffset); // https://www.desmos.com/calculator/rqqamhfeek
        double Direction2 = Math.sin(theta - Math.PI/4 - YawOffset); // https://www.desmos.com/calculator/dminewe5vs

        double leftFrontPower  = Direction1;
        double rightBackPower  = Direction1;
        double leftBackPower   = Direction2;
        double rightFrontPower = Direction2;

        leftFrontDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFrontDrive.setPower(leftFrontPower);

        rightFrontDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setPower(rightFrontPower);

        rightBackDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setPower(rightBackPower);

        leftBackDrive.setTargetPosition(Pos); // Tells the motor that the position it should go to is desiredPosition
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setPower(leftBackPower);
    }
}