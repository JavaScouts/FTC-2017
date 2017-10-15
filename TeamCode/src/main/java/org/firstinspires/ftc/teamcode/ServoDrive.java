package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.*;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo")
public class ServoDrive extends OpMode {

    Servo s1;



    public void init(){

        s1 = hardwareMap.servo.get("s1");

    }


    public void loop() {


        if (gamepad1.a){
            s1.setPosition(1.0);
        }

        if (gamepad1.b) {
            s1.setPosition(0);
        }

        telemetry.addData("Servo Position", s1.getPosition());
    }

}
