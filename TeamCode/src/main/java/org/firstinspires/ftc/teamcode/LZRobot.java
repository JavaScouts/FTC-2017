package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.sql.Time;
import java.util.ArrayList;
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
   // public Servo s1;
    public Servo s2;
    public DcMotor slide;
    public DcMotor Arm;
    public ModernRoboticsI2cRangeSensor range1;
    public ModernRoboticsI2cRangeSensor range2;
    public ColorSensor color;
    public float Y1;
    public float X1;
    public float X2;
    public int[] positions = {};
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
        leftDrive = map.dcMotor.get("fl");
        rightDrive = map.dcMotor.get("fr");
        backLDrive = map.dcMotor.get("bl");
        backRDrive = map.dcMotor.get("br");
        slide = map.dcMotor.get("slider");
        Arm = map.dcMotor.get("Arm");

        myOpMode.telemetry.addData("Initialization:", "Motors Initialized.");
        myOpMode.telemetry.update();

        //s1 = map.servo.get("s1");
        s2 = map.servo.get("s2");

        myOpMode.telemetry.addData("Initialization:", "Servos Initialized.");
        myOpMode.telemetry.update();

        color = map.colorSensor.get("Color");
        //range1 = map.get(ModernRoboticsI2cRangeSensor.class, "range1");
        //range2 = map.get(ModernRoboticsI2cRangeSensor.class, "range2");

        myOpMode.telemetry.addData("Initialization:", "Sensors Initialized.");
        myOpMode.telemetry.update();

        //set directions
        backRDrive.setDirection(DcMotor.Direction.REVERSE);
        //s1.setDirection(Servo.Direction.REVERSE);

        //STOP EVERYTHING
        moveRobot(0, 0, 0);
        //s1.setPosition(0.4);
        s2.setPosition(0);
        myOpMode.telemetry.addData("Initialization:", "Complete!");
        myOpMode.telemetry.update();

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
    public void rotateTime(String dir, double power, long time)
    {

        //self explanatory rotation for mecanum wheels
        if(dir.equals("cclock")) {

            backLDrive.setPower(-power);
            leftDrive.setPower(-power);
            backRDrive.setPower(power);
            rightDrive.setPower(power);
            try {
                TimeUnit.SECONDS.sleep(time);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

        } else if(dir.equals("clock")) {

            backLDrive.setPower(power);
            leftDrive.setPower(power);
            backRDrive.setPower(-power);
            rightDrive.setPower(-power);
            try {
                TimeUnit.SECONDS.sleep(time);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

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

            moveRobot(0,0,0);

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
            moveRobot(0,0,0);

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
            moveRobot(0,0,0);

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
            moveRobot(0,0,0);

        }

    }
    
    public void manualDrive()  {
        // In this mode the Left stick moves the robot fwd & back, and Right & Left.
        // The Right stick rotates CCW and CW.

        //  (note: The joystick goes negative when pushed forwards, so negate it)
        setAxial(-myOpMode.gamepad1.left_stick_y);
        setLateral(myOpMode.gamepad1.left_stick_x);
        setYaw(myOpMode.gamepad1.right_stick_x);

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

    public void dpadDrive(double power) {

        if(myOpMode.gamepad1.dpad_left) {

            backLDrive.setPower(power);
            leftDrive.setPower(-power);
            backRDrive.setPower(-power);
            rightDrive.setPower(power);

        } else if(myOpMode.gamepad1.dpad_right) {

            backLDrive.setPower(-power);
            leftDrive.setPower(power);
            backRDrive.setPower(power);
            rightDrive.setPower(-power);

        } else if(myOpMode.gamepad1.dpad_up) {

            backLDrive.setPower(power);
            leftDrive.setPower(power);
            backRDrive.setPower(power);
            rightDrive.setPower(power);

        } else if(myOpMode.gamepad1.dpad_down) {

            backLDrive.setPower(-power);
            leftDrive.setPower(-power);
            backRDrive.setPower(-power);
            rightDrive.setPower(-power);

        }
    }

    public void setAxial(double axial)      {driveAxial = Range.clip(axial, -1, 1);}
    public void setLateral(double lateral)  {driveLateral = Range.clip(lateral, -1, 1); }
    public void setYaw(double yaw)          {driveYaw = Range.clip(yaw, -1, 1); }

    public void setAxialtoLateral(double axial)          {driveLateral = Range.clip(axial, -1, 1);}
    public void setLateraltoAxial(double lateral)        {driveAxial = -Range.clip(lateral, -1, 1);}
    public void setChangeYaw(double yaw)                 {driveYaw = Range.clip(yaw, -1, 1);}

    public void setMode(DcMotor.RunMode mode ) {

        leftDrive.setMode(mode);
        rightDrive.setMode(mode);
        backLDrive.setMode(mode);
        backRDrive.setMode(mode);
        slide.setMode(mode);

    }

    public int[] getCurrentPosition() {

        positions = new int[]{

                leftDrive.getCurrentPosition(),

                rightDrive.getCurrentPosition(),

                backLDrive.getCurrentPosition(),

                backRDrive.getCurrentPosition(),

                slide.getCurrentPosition(),

        };

        return positions;

    }

    /*  motor integer numbers for \/ getCurrentPosition(int motor)
            1(l)        2(r)
                5(slide)



            3(bl)       4(br)

     */

    public int getCurrentPosition(int motor) {

        motor = motor-1;

        return positions[motor];

    }
}
