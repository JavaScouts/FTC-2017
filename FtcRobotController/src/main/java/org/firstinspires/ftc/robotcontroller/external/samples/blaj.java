package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Robotics Team 9986 on 11/20/2016.
 */
//@TeleOp(name = "Blaj")
public class blaj extends OpMode {

    DcMotor Cool;
    DcMotor Cool2;

    public void init() {

        Cool = hardwareMap.dcMotor.get("left");
        Cool2 = hardwareMap.dcMotor.get("right");
    }

    public void loop() {


        float leftjoy = -gamepad1.left_stick_y;
        float rightjoy = -gamepad1.right_stick_y;

        Cool.setPower(leftjoy * 0.5);
        Cool2.setPower(rightjoy * 0.5);


    }
}
