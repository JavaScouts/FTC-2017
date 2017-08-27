package org.firstinspires.ftc.robotcontroller.external.samples;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by KyleG on 1/3/2016.5.
 */@TeleOp(name = "Arm")
public class servo extends OpMode {

    DcMotor Arm;
    double arm_position = 0;




    public void init() {

        Arm = hardwareMap.dcMotor.get("Arm");
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    public void loop() {




        if(gamepad2.y && arm_position == 0) {

            arm_position = 1;
            Arm.setTargetPosition(100);
            Arm.setPower(1);


        }
        if (gamepad2.b && arm_position == 1){

            arm_position = 0;
            Arm.setTargetPosition(0);
            Arm.setPower(1);

        }
    }
}




