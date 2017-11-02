package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.concurrent.TimeUnit;

import static java.lang.Math.abs;

/**
 * Created by seed on 9/11/17.
 */

public class LZRobot {

    //initialize motors
    public DcMotor leftDrive;
    public DcMotor rightDrive;
    public DcMotor backLDrive;
    public DcMotor backRDrive;
    public DcMotor Coll;
    public DcMotor Arm;
    public float Y1;
    public float X1;
    public float X2;
    private double  driveAxial      = 0 ;   // Positive is forward
    private double  driveLateral    = 0 ;   // Positive is right
    private double  driveYaw        = 0 ;   // Positive is CCW
    //public Servo servo1;
    private LinearOpMode myOpMode;
    //create map
    HardwareMap map;

    public LZRobot() {

    }


    public void init(HardwareMap map1, LinearOpMode opMode) {

        map = map1;
        myOpMode = opMode;

        //get motors from map
        leftDrive = map.dcMotor.get("FrontMotor1");
        rightDrive = map.dcMotor.get("FrontMotor2");
        backLDrive = map.dcMotor.get("BackMotor1");
        backRDrive = map.dcMotor.get("BackMotor2");
        //Coll = map.dcMotor.get("Collector");
        Arm = map.dcMotor.get("Arm");
        //servo1 = map.servo.get("s1");

        //set directions
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        backLDrive.setDirection(DcMotor.Direction.REVERSE);
        backRDrive.setDirection(DcMotor.Direction.FORWARD);

        //stop
        //stopRobot();
        moveRobot(0,0,0);
    }
    
    public void stopRobot() {

        //self explanatory
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        backLDrive.setPower(0);
        backRDrive.setPower(0);
        //Coll.setPower(0);
        Arm.setPower(0);

    }

    //function move takes a direction and a power
    //probably a better way to do this
    public void move(String dir, float power) {

        //self explanatory
        switch (dir) {
            case "left":

                backLDrive.setPower(power);
                leftDrive.setPower(-power);
                backRDrive.setPower(-power);
                rightDrive.setPower(power);

                break;
            case "right":

                backLDrive.setPower(-power);
                leftDrive.setPower(power);
                backRDrive.setPower(power);
                rightDrive.setPower(-power);

                break;
            case "forwards":

                backLDrive.setPower(power);
                leftDrive.setPower(power);
                backRDrive.setPower(power);
                rightDrive.setPower(power);

                break;
            case "backwards":

                backLDrive.setPower(-power);
                leftDrive.setPower(-power);
                backRDrive.setPower(-power);
                rightDrive.setPower(-power);

                break;
        }

    }

    //rotate function, takes direction and power
    public void rotate(String dir, double power) {

        //self explanatory rotation for mecanum wheels
        if(dir.equals("cclock")) {

            backLDrive.setPower(-power);
            leftDrive.setPower(-power);
            backRDrive.setPower(power);
            rightDrive.setPower(power);

        } else if(dir.equals("clock")) {

            backLDrive.setPower(power);
            leftDrive.setPower(power);
            backRDrive.setPower(-power);
            rightDrive.setPower(-power);

        }

    }

    public void mecDrive(float leftY, float leftX, float rightX, double threshold) {

        if (abs(leftY) > threshold) {
            Y1 = leftY;
        } else {
            Y1 = 0;
        }
        if (abs(leftX) > threshold) {
            X1 = leftX;
        } else {
            X1 = 0;
        }
        if (abs(rightX) > threshold) {
            X2 = rightX;
        } else {
            X2 = 0;
        }

        leftDrive.setPower(Y1 - X2 - X1);
        backLDrive.setPower(Y1 - X2 + X1);
        rightDrive.setPower(Y1 + X2 + X1);
        backRDrive.setPower(Y1 + X2 - X1);

    }

    public void moveTime(String dir, double power, long timeS) {

        if(dir == "left") {

            backLDrive.setPower(power);
            leftDrive.setPower(-power);
            backRDrive.setPower(-power);
            rightDrive.setPower(power);
            try {
                TimeUnit.SECONDS.sleep(timeS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopRobot();

        } else if(dir == "right") {

            backLDrive.setPower(-power);
            leftDrive.setPower(power);
            backRDrive.setPower(power);
            rightDrive.setPower(-power);
            try {
                TimeUnit.SECONDS.sleep(timeS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopRobot();

        } else if(dir == "forwards") {

            backLDrive.setPower(power);
            leftDrive.setPower(power);
            backRDrive.setPower(power);
            rightDrive.setPower(power);
            try {
                TimeUnit.SECONDS.sleep(timeS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopRobot();

        } else if(dir == "backwards") {

            backLDrive.setPower(-power);
            leftDrive.setPower(-power);
            backRDrive.setPower(-power);
            rightDrive.setPower(-power);
            try {
                TimeUnit.SECONDS.sleep(timeS);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            stopRobot();

        }

    }
    
    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(-myOpMode.gamepad1.right_stick_x);
    }
    
    public void newManualDrive() {

        double r = Math.hypot(-myOpMode.gamepad1.left_stick_x, -myOpMode.gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-myOpMode.gamepad1.left_stick_y, -myOpMode.gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = -myOpMode.gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        leftDrive.setPower(v1);
        rightDrive.setPower(v2);
        backLDrive.setPower(v3);
        backRDrive.setPower(v4);
        
    }

    public void moveRobot(double axial, double lateral, double yaw) {
        setAxial(axial);
        setYaw(yaw);
        setLateral(lateral);
        moveRobot();
    }

    public void moveRobot() {
        // calculate required motor speeds to acheive axis motions
        double backL = driveAxial - driveLateral + driveYaw;
        double backR = driveAxial + driveLateral - driveYaw;
        double left = driveAxial + driveLateral + driveYaw;
        double right = driveAxial - driveLateral - driveYaw;

        // normalize all motor speeds so no values exceeds 100%.
        double max = Math.max(Math.abs(left), Math.abs(right));
        max = Math.max(max, Math.abs(backL));
        max = Math.max(max, Math.abs(backR));
        if (max > 1.0)
        {
            backR /= max;
            backL /= max;
            right /= max;
            left /= max;
        }

        // Set drive motor power levels.
        backLDrive.setPower(backL);
        backRDrive.setPower(backR);
        leftDrive.setPower(left);
        rightDrive.setPower(right);

        // Display Telemetry
        myOpMode.telemetry.addData("Axes  ", "A[%+5.2f], L[%+5.2f], Y[%+5.2f]", driveAxial, driveLateral, driveYaw);
        myOpMode.telemetry.addData("Wheels", "L[%+5.2f], R[%+5.2f], BL[%+5.2f], BR[%+5.2f]", left, right, backL, backR);
    }

    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }

    public void setMode(DcMotor.RunMode mode ) {
        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
        backLDrive.setMode(mode);
        backRDrive.setMode(mode);

    }

}
