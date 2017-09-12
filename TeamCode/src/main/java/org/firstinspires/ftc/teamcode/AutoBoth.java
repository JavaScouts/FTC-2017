package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


/**
 * Created by Robotics Team 9986 on 12/3/2016.
 */
//@Autonomous(name = "AutoBoth", group = "AutoBoth")
public class AutoBoth extends LinearOpMode {
    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor Coll;
    DcMotor Arm;

    double leftPower, rightPower, correction;
    final double PERFECT_COLOR_VALUE = 0.19;
    final double SCALE_VALUE = .25;
    final double FOLLOW_POWER = 0.15;

    OpticalDistanceSensor lightSensor;

    static final double     WHITE_THRESHOLD = 0.5;
    static final double     APPROACH_SPEED  = 0.25;


    @Override
    public void runOpMode() throws InterruptedException {

        /* Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        FrontMotor1 = hardwareMap.dcMotor.get("FrontMotor1");
        FrontMotor2 = hardwareMap.dcMotor.get("FrontMotor2");
        Coll = hardwareMap.dcMotor.get("Collector");
        Arm = hardwareMap.dcMotor.get("Arm");

        BackMotor1 = hardwareMap.dcMotor.get("BackMotor1");
        BackMotor2 = hardwareMap.dcMotor.get("BackMotor2");

        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.REVERSE);
        BackMotor1.setDirection(DcMotor.Direction.FORWARD);
        BackMotor2.setDirection(DcMotor.Direction.REVERSE);


        lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");  // Alternative MR ODS sensor.

        lightSensor.enableLed(true);

        telemetry.addData("Color Value", lightSensor.getLightDetected());

        waitForStart();

        while (!(isStarted() || isStopRequested())) {

            telemetry.addData("Light Level", lightSensor.getLightDetected());
            telemetry.update();
        }

        FrontMotor1.setPower(APPROACH_SPEED);
        BackMotor1.setPower(APPROACH_SPEED);
        FrontMotor2.setPower(APPROACH_SPEED);
        BackMotor2.setPower(APPROACH_SPEED);

        while (opModeIsActive() && (lightSensor.getLightDetected() <= WHITE_THRESHOLD)) {

            telemetry.addData("Light Level",  lightSensor.getLightDetected());
            telemetry.update();
        }

        FrontMotor1.setPower(0);
        BackMotor1.setPower(0);
        FrontMotor2.setPower(0);
        BackMotor2.setPower(0);


        sleep(500);

        while (opModeIsActive()) {
            // Get a correction
            correction = (PERFECT_COLOR_VALUE - lightSensor.getLightDetected()) * SCALE_VALUE;
            telemetry.addData("Color Value", lightSensor.getLightDetected());
            telemetry.addData("Correction", correction);
            telemetry.update();


            // Sets the powers so they are no less than .075 and apply to correction
            if (correction <= 0) {
                leftPower = FOLLOW_POWER - correction;
                rightPower = FOLLOW_POWER;
            } else {
                leftPower = FOLLOW_POWER;
                rightPower = FOLLOW_POWER + correction;
            }

            // Sets the powers to the motors
            FrontMotor1.setPower(leftPower);
            BackMotor1.setPower(leftPower);
            FrontMotor2.setPower(rightPower);
            BackMotor2.setPower(rightPower);
        }

    }
}

