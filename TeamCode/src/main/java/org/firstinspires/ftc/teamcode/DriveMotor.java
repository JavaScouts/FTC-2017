package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Robotics Team 9986 on 7/24/2016.
 */
public class DriveMotor extends OpMode {

    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor Arm;


    public void init(){

        FrontMotor1 = hardwareMap.dcMotor.get("FrontMotor1");
        FrontMotor2 = hardwareMap.dcMotor.get("FrontMotor2");

        Arm = hardwareMap.dcMotor.get("Arm");

        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.FORWARD);
        Arm.setDirection(DcMotor.Direction.FORWARD);


    }

    public void loop() {

         float arm = -gamepad2.right_stick_x;
        Arm.setPower(arm);





        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;


        FrontMotor1.setPower(leftY);
        FrontMotor2.setPower(rightY);
    }

}
