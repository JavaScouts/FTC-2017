package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

/**
 * Created by seed on 9/11/17.
 */

public class LZTestChassis {

    //initialize motors
    public DcMotor FrontMotor1;
    public DcMotor FrontMotor2;
    public DcMotor BackMotor1;
    public DcMotor BackMotor2;
    public Servo servo1;
    public Servo servo2;

    //create map
    HardwareMap map;

    public void init(HardwareMap map1) {

        map = map1;

        //get motors from map
        FrontMotor1 = map.dcMotor.get("Left");
        FrontMotor2 = map.dcMotor.get("Right");
        BackMotor1 = map.dcMotor.get("Left1");
        BackMotor2 = map.dcMotor.get("Right1");
        servo1 = map.servo.get("s1");
        servo2 = map.servo.get("s2");

        //set directions
        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.REVERSE);
        BackMotor1.setDirection(DcMotor.Direction.FORWARD);
        BackMotor2.setDirection(DcMotor.Direction.REVERSE);

        //stop
        stopRobot();

    }

    public void stopRobot() {

        //self explanatory
        FrontMotor1.setPower(0);
        FrontMotor2.setPower(0);
        BackMotor1.setPower(0);
        BackMotor2.setPower(0);

    }

    //function move takes a direction and a power
    //probably a better way to do this
    void move(String dir, float power) {

        //self explanatory
        switch (dir) {
            case "left":

                BackMotor1.setPower(power);
                FrontMotor1.setPower(-power);
                BackMotor2.setPower(-power);
                FrontMotor2.setPower(power);

                break;
            case "right":

                BackMotor1.setPower(-power);
                FrontMotor1.setPower(power);
                BackMotor2.setPower(power);
                FrontMotor2.setPower(-power);

                break;
            case "forwards":

                BackMotor1.setPower(power);
                FrontMotor1.setPower(power);
                BackMotor2.setPower(power);
                FrontMotor2.setPower(power);

                break;
            case "backwards":

                BackMotor1.setPower(-power);
                FrontMotor1.setPower(-power);
                BackMotor2.setPower(-power);
                FrontMotor2.setPower(-power);

                break;
        }

    }

    //rotate function, takes direction and power
    public void rotate(String dir, double power) {

        //self explanatory rotation for mecanum wheels
        if(dir.equals("cclock")) {

            BackMotor1.setPower(-power);
            FrontMotor1.setPower(-power);
            BackMotor2.setPower(power);
            FrontMotor2.setPower(power);

        } else if(dir.equals("clock")) {

            BackMotor1.setPower(power);
            FrontMotor1.setPower(power);
            BackMotor2.setPower(-power);
            FrontMotor2.setPower(-power);

        }

    }

    public void moveTime(String dir, double power, long timeS) {

        if(dir == "left") {

            BackMotor1.setPower(power);
            FrontMotor1.setPower(-power);
            BackMotor2.setPower(-power);
            FrontMotor2.setPower(power);
            try {
                TimeUnit.SECONDS.sleep(timeS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopRobot();

        } else if(dir == "right") {

            BackMotor1.setPower(-power);
            FrontMotor1.setPower(power);
            BackMotor2.setPower(power);
            FrontMotor2.setPower(-power);
            try {
                TimeUnit.SECONDS.sleep(timeS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopRobot();

        } else if(dir == "forwards") {

            BackMotor1.setPower(power);
            FrontMotor1.setPower(power);
            BackMotor2.setPower(power);
            FrontMotor2.setPower(power);
            try {
                TimeUnit.SECONDS.sleep(timeS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopRobot();

        } else if(dir == "backwards") {

            BackMotor1.setPower(-power);
            FrontMotor1.setPower(-power);
            BackMotor2.setPower(-power);
            FrontMotor2.setPower(-power);
            try {
                TimeUnit.SECONDS.sleep(timeS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopRobot();

        }

    }

}
