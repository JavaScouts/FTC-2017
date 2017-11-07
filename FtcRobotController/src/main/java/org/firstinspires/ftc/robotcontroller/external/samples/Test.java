package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by wadno on 9/9/2017.
 */
@TeleOp(name = "Test")
public class Test extends OpMode {
    DcMotor A;
    DcMotor B;
    DcMotor C;
    DcMotor D;
    Servo servo;

    public void init() {

        A = hardwareMap.dcMotor.get("A");
        B = hardwareMap.dcMotor.get("B");
        C = hardwareMap.dcMotor.get("C");
        D = hardwareMap.dcMotor.get("D");
        servo = hardwareMap.servo.get("servo");
    }

    public void loop() {

        if (gamepad2.x){
            servo.setPosition(0.5);
        } else if(gamepad2.b){
            servo.setPosition(0);
        }

        float leftstick = -gamepad1.left_stick_y;
        float rightstick = -gamepad1.right_stick_y;

        A.setPower(leftstick);
        C.setPower(leftstick);
        B.setPower(rightstick);
        D.setPower(rightstick);


    }
}
