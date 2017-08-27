package org.firstinspires.ftc.robotcontroller.external.samples;


import android.graphics.Color;
import android.service.carrier.CarrierService;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by kyle on 10/16/2015.
 */
@Autonomous(name = "Wall")
public class FollowWall extends OpMode {
    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor Arm;
    double leftFrontPower = 0;
    double leftBackPower = 0;
    double rightFrontPower = 0;
    double rightBackPower = 0;
    double tolerance = 0.25;
    double Gain = 0.05;
    ModernRoboticsI2cRangeSensor rangeSensor;

    OpticalDistanceSensor ods;


    ElapsedTime time;

    enum State {Follower, Straight, done}

    ;
    State state;

    @Override
    public void init() {

        FrontMotor1 = hardwareMap.dcMotor.get("FrontMotor1");
        FrontMotor2 = hardwareMap.dcMotor.get("FrontMotor2");

        BackMotor1 = hardwareMap.dcMotor.get("BackMotor1");
        BackMotor2 = hardwareMap.dcMotor.get("BackMotor2");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");

        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.REVERSE);
        BackMotor1.setDirection(DcMotor.Direction.FORWARD);
        BackMotor2.setDirection(DcMotor.Direction.REVERSE);
        ods = hardwareMap.opticalDistanceSensor.get("ods");  // Alternative MR ODS sensor.

        ods.enableLed(true);


        time = new ElapsedTime();
        state = State.Follower;

    }

    @Override
    public void loop() {
        double error = rangeSensor.getDistance(DistanceUnit.INCH) - 15;

        switch (state) {
            case Follower:
                if (error < tolerance)  //The robot is too close to the wall
                {
                    leftFrontPower = -Gain * error;
                    leftBackPower = Gain * error;
                    rightFrontPower = Gain * error;
                    rightBackPower = -Gain * error;
                } else if (error > tolerance)  //The robot is too far away
                {
                    leftFrontPower = Gain * error;
                    leftBackPower = -Gain * error;
                    rightFrontPower = -Gain * error;
                    rightBackPower = Gain * error;
                } else {
                    leftFrontPower = 0.5;
                    leftBackPower = 0.5;
                    rightFrontPower = 0.5;
                    rightBackPower = 0.5;
                }
                BackMotor1.setPower(-leftBackPower);
                FrontMotor1.setPower(leftFrontPower);
                BackMotor2.setPower(rightBackPower);
                FrontMotor2.setPower(-rightFrontPower);
                if (ods.getLightDetected() > 0.5) {

                    state = State.done;

                }
                BackMotor1.setPower(Range.clip(leftBackPower,-0.6,0.6));
                FrontMotor1.setPower(Range.clip(leftFrontPower,-0.6,0.6));
                BackMotor2.setPower(Range.clip(rightBackPower,-0.6,0.6));
                FrontMotor2.setPower(Range.clip(rightFrontPower,-0.6,0.6));
                break;

            case done:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
        }

    }
}
