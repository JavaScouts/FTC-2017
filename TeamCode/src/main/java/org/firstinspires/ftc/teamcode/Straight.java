package org.firstinspires.ftc.teamcode;


import android.service.carrier.CarrierService;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by kyle on 10/16/2015.
 */
@Autonomous(name = "Straight")
@Disabled
public class Straight extends OpMode {
    //OpticalDistanceSensor Line;

    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor Arm;

    ElapsedTime time;

    static final double wait = 5.0;

    static final double forward = 2.5;



    enum State {wait, Straight, done}

    ;
    State state;

    @Override
    public void init() {

        FrontMotor1 = hardwareMap.dcMotor.get("FrontMotor1");
        FrontMotor2 = hardwareMap.dcMotor.get("FrontMotor2");
        Arm = hardwareMap.dcMotor.get("Arm");

        BackMotor1 = hardwareMap.dcMotor.get("BackMotor1");
        BackMotor2 = hardwareMap.dcMotor.get("BackMotor2");

        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.REVERSE);
        BackMotor1.setDirection(DcMotor.Direction.FORWARD);
        BackMotor2.setDirection(DcMotor.Direction.REVERSE);

        time = new ElapsedTime();
        state = State.wait;




    }


    @Override
    public void loop() {

        double currentTime = time.time();

        switch (state) {

            case wait:

                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
                Arm.setPower(0);

                if (currentTime > wait) {

                    state = State.Straight;
                    time.reset();
                }
                break;

            case Straight:

                FrontMotor1.setPower(100);
                FrontMotor2.setPower(100);
                BackMotor1.setPower(100);
                BackMotor2.setPower(100);

                if (currentTime > forward) {

                    state = State.done;
                    time.reset();
                }
                break;


            case done:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
                Arm.setPower(0);



        }
    }
}



