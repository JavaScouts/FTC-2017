package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics Team 9986 on 11/20/2016.
 */
@TeleOp(name = "Picasso")
public class Picasso extends OpMode {

    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    Servo servo1;

    public void init() {

        FrontMotor1 = hardwareMap.dcMotor.get("FrontMotor1");
        FrontMotor2 = hardwareMap.dcMotor.get("FrontMotor2");
        servo1 = hardwareMap.servo.get("servo1");

    }

    public void loop() {

        if(gamepad1.a){
            servo1.setPosition(0.5);
        }
        else if (gamepad1.b){
            servo1.setPosition(0.1);
        }

        float leftjoy = -gamepad1.left_stick_y;
        float rightjoy = -gamepad1.right_stick_y;

        FrontMotor1.setPower(leftjoy);
        FrontMotor2.setPower(-rightjoy);


    }
}
