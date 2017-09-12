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
@Autonomous(name = "Final Autonomous")
@Disabled
public class Autonomous5 extends OpMode {
    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor Arm;

    ElapsedTime time;


    static final double forward = 5.5;
    static final double linee = 0.5;
    static final double Linee2 = 0.5;
    static final double wait = 1.5;
    static final double right = 0.37;
    static final double forward2 = 0.9;



    enum State { Init, Straight, Follower, pause, Follower2, turn, go, done}

    ;
    State state;

    @Override
    public void init() {

        FrontMotor1 = hardwareMap.dcMotor.get("FrontMotor1");
        FrontMotor2 = hardwareMap.dcMotor.get("FrontMotor2");

        BackMotor1 = hardwareMap.dcMotor.get("BackMotor1");
        BackMotor2 = hardwareMap.dcMotor.get("BackMotor2");

        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.REVERSE);
        BackMotor1.setDirection(DcMotor.Direction.FORWARD);
        BackMotor2.setDirection(DcMotor.Direction.REVERSE);




    }
    @Override
    public void start(){
        time = new ElapsedTime();

        time.reset();
        state = State.Init;

    }


    @Override
    public void loop() {

        double currentTime = time.time();

        switch (state) {

            case Init:
                FrontMotor1.setPower(100);
                FrontMotor2.setPower(100);
                BackMotor1.setPower(100);
                BackMotor2.setPower(100);

                if (currentTime > 0.5 ) {

                    state = State.Follower;
                    time.reset();
                }
                break;

            case Straight:

                FrontMotor1.setPower(100);
                FrontMotor2.setPower(100);
                BackMotor1.setPower(100);
                BackMotor2.setPower(100);

                if (currentTime > 0.5 ) {

                    state = State.Follower;
                    time.reset();
                }
                break;

            case Follower:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);

                        if (currentTime > 0.5 ) {

                            state = State.done;
                            time.reset();
                        }
                break;


            case done:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);



        }
    }
}



