package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Liam on 11/16/2017.
 */

public class test extends LinearOpMode {

    DcMotor leftMotor;
    DcMotor rightMotor;
    DcMotor arm;
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("leftMotor");
        rightMotor = hardwareMap.dcMotor.get("rightMotor");
        servo = hardwareMap.servo.get("servo");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {

            float lY = -gamepad1.left_stick_y;
            float ry = -gamepad1.right_stick_y;

            leftMotor.setPower(lY);
            rightMotor.setPower(ry);

            if(gamepad1.x) {

                servo.setPosition(0.5);

            } else {

                servo.setPosition(0);

            }
        }
    }
}
