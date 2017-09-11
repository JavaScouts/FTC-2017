package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Created by seed on 9/11/17.
 */

public class LZRobot {

    //initialize motors
    public DcMotor FrontMotor1;
    public DcMotor FrontMotor2;
    public DcMotor BackMotor1;
    public DcMotor BackMotor2;
    public DcMotor Coll;
    public DcMotor Arm;

    //create map
    HardwareMap map;

    public void init(HardwareMap map1) {

        map = map1;

        //get motors from map
        FrontMotor1 = map.dcMotor.get("FrontMotor1");
        FrontMotor2 = map.dcMotor.get("FrontMotor2");
        BackMotor1 = map.dcMotor.get("BackMotor1");
        BackMotor2 = map.dcMotor.get("BackMotor2");
        Coll = map.dcMotor.get("Collector");
        Arm = map.dcMotor.get("Arm");

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
        Coll.setPower(0);
        Arm.setPower(0);

    }

    //function move takes a direction and a power
    //probably a better way to do this
    public void move(String dir, double power) {

        //self explanatory
        if(dir == "left") {

            BackMotor1.setPower(power);
            FrontMotor1.setPower(-power);
            BackMotor2.setPower(-power);
            FrontMotor2.setPower(power);

        } else if(dir == "right") {

            BackMotor1.setPower(-power);
            FrontMotor1.setPower(power);
            BackMotor2.setPower(power);
            FrontMotor2.setPower(-power);

        } else if(dir == "forwards") {

            BackMotor1.setPower(power);
            FrontMotor1.setPower(power);
            BackMotor2.setPower(power);
            FrontMotor2.setPower(power);

        } else if(dir == "backwards") {

            BackMotor1.setPower(-power);
            FrontMotor1.setPower(-power);
            BackMotor2.setPower(-power);
            FrontMotor2.setPower(-power);

        }

    }

    //rotate function, takes direction and power
    public void rotate(String dir, double power) {

        //self explanatory rotation for mecanum wheels
        if(dir == "cclock") {

            BackMotor1.setPower(-power);
            FrontMotor1.setPower(-power);
            BackMotor2.setPower(power);
            FrontMotor2.setPower(power);

        } else if(dir == "clock") {

            BackMotor1.setPower(power);
            FrontMotor1.setPower(power);
            BackMotor2.setPower(-power);
            FrontMotor2.setPower(-power);

        }

    }

}
