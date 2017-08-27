package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Strafe")
public class ServoDrive extends OpMode {

    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor Arm;




    public void init(){

        FrontMotor1 = hardwareMap.dcMotor.get("FrontMotor1");
        FrontMotor2 = hardwareMap.dcMotor.get("FrontMotor2");

        BackMotor1 = hardwareMap.dcMotor.get("BackMotor1");
        BackMotor2 = hardwareMap.dcMotor.get("BackMotor2");

        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.REVERSE);
        BackMotor1.setDirection(DcMotor.Direction.FORWARD);
        BackMotor2.setDirection(DcMotor.Direction.REVERSE);
        Arm = hardwareMap.dcMotor.get("Arm");

    }


    public void loop() {
        //Goes left
        while (gamepad1.b) {
            BackMotor1.setPower(100);
            FrontMotor1.setPower(-100);
            BackMotor2.setPower(-100);
            FrontMotor2.setPower(100);

        }
        //Goes Right
        while (gamepad1.x) {
            BackMotor1.setPower(-100);
            FrontMotor1.setPower(100);
            BackMotor2.setPower(100);
            FrontMotor2.setPower(-100);

        }

       while (gamepad1.y) {
           Arm.setPower(100);
       }


        float leftY = -gamepad1.left_stick_y;
        float rightY = -gamepad1.right_stick_y;

        FrontMotor1.setPower(leftY);
        BackMotor1.setPower(leftY);
        FrontMotor2.setPower(rightY);
        BackMotor2.setPower(rightY);
    }

}
