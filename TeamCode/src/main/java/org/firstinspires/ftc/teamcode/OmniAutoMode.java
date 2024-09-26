package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoDickstein")
public class OmniAutoMode extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront"); //Motor 0 = left front
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront"); //Motor 1 = right front
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack"); //Motor 2 = right bottom
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack"); //Motor 3 = left bottom

    }
