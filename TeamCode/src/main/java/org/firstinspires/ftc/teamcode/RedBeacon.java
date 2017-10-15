package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by Robotics Team 9986 on 1/2/2017.
 */
@Autonomous(name = "RedBeacon")
@Disabled

public class RedBeacon extends OpMode {
    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor Arm;
    DcMotor coll;
    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor ods;
   /* double leftFrontPower = 0;
    double leftBackPower = 0;
    double rightFrontPower = 0;
    double rightBackPower = 0;
    double tolerance = 0.25;
    double Gain = 0.05;*/




    ElapsedTime time;


    enum State {Init, Straight1, backwar, forward, Shoot, wCollect, wshoot, backwards,  Turn1, range1, backward2, wallfollow, wait7, hit, backwards3, check, wait, secondstrafe, secondstrafe2, hit2, backwards4, check2, wait2, done}
    ;

    State state;
    @Override
    public void init() {

        FrontMotor1 = hardwareMap.dcMotor.get("FrontMotor1");
        FrontMotor2 = hardwareMap.dcMotor.get("FrontMotor2");
        Arm = hardwareMap.dcMotor.get("Arm");
        colorSensor = hardwareMap.colorSensor.get("color");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range");
        ods = hardwareMap.opticalDistanceSensor.get("ods");

        BackMotor1 = hardwareMap.dcMotor.get("BackMotor1");
        BackMotor2 = hardwareMap.dcMotor.get("BackMotor2");
        coll = hardwareMap.dcMotor.get("Collector");

        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.REVERSE);
        BackMotor1.setDirection(DcMotor.Direction.FORWARD);
        BackMotor2.setDirection(DcMotor.Direction.REVERSE);
        colorSensor.enableLed(false);
        rangeSensor.enableLed(false);

        ods.enableLed(true);
        //time = new ElapsedTime();
        //time.reset();
        //state = State.Init;





    }
    @Override
     public void start(){
        time = new ElapsedTime();
        time.reset();
        state = State.Init;


    }


    @Override
    public void loop() {
        telemetry.addData("ColorSensor", colorSensor.red());
        telemetry.addData("ColorSensor2Blue", colorSensor.blue());
        telemetry.addData("Range", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("ODS", ods.getLightDetected());
        double currentTime = time.time();
      //  double error = rangeSensor.getDistance(DistanceUnit.INCH) - 8;


        switch (state) {
            case Init:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);

                if (currentTime > 0.2) {
                    time.reset();

                    state = State.Straight1;

                }
                break;
            case Straight1:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);

                if (currentTime > 0.5)  {

                    state = State.forward;
                    time.reset();
                }

                break;
            case backwar:
                if (rangeSensor.getDistance(DistanceUnit.CM) > 0.5) {
                    FrontMotor1.setPower(0.5);
                    FrontMotor2.setPower(0.5);
                    BackMotor1.setPower(0.5);
                    BackMotor2.setPower(0.5);
                } else{
                        state = State.forward;
                        time.reset();
                    }


                break;
            case forward:
                Arm.setPower(0);
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);
                if (currentTime > 0.5)  {

                    state = State.Shoot;
                    time.reset();
                }
                break;

            case Shoot:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);

                Arm.setPower(1.0);

                if (currentTime > 0.5)  {
                    Arm.setPower(0);

                    state = State.wCollect;
                    time.reset();
                }
                break;
            case wCollect:
                Arm.setPower(0);
                coll.setPower(1.0);
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);

                if (currentTime > 2.0){
                    coll.setPower(0);
                    time.reset();
                    state = State.wshoot;
                }
                break;
            case wshoot:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);

                Arm.setPower(0.9);

                if (currentTime > 0.5)  {
                    Arm.setPower(0);

                    state = State.backwards;
                    time.reset();
                }
                break;

            case backwards:
                FrontMotor1.setPower(0.5);
                FrontMotor2.setPower(0.5);
                BackMotor1.setPower(0.5);
                BackMotor2.setPower(0.5);


                if (currentTime > 0.01)  {

                    state = State.Turn1;
                    time.reset();
                }

                break;
            case Turn1:
                Arm.setPower(0);
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(0);
                BackMotor2.setPower(-0.5);
                if (currentTime > 1.85)  {

                    state = State.range1;
                    time.reset();
                }
                break;


            case range1:
                if (rangeSensor.getDistance(DistanceUnit.CM) > 5)
                {
                    FrontMotor1.setPower(0.2);
                    FrontMotor2.setPower(0.2);
                    BackMotor1.setPower(0.2);
                    BackMotor2.setPower(0.2);
                }
                else  {

                    state = State.backward2;
                    time.reset();
                }

                break;
            case backward2:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);




                if (currentTime > 0.3)  {

                    state = State.wallfollow;
                    time.reset();
                }

                break;


            case wallfollow:
                BackMotor1.setPower(-0.25);
                FrontMotor1.setPower(0.25);
                BackMotor2.setPower(0.25);
                FrontMotor2.setPower(-0.25);

                if (ods.getLightDetected() > 0.5) {

                    state = State.hit;
                    time.reset();

                }
                break;
            case hit:
                FrontMotor1.setPower(1.0);
                FrontMotor2.setPower(1.0);
                BackMotor1.setPower(1.0);
                BackMotor2.setPower(1.0);


                if (currentTime > 0.94)  {

                    state = State.backwards3;
                    time.reset();
                }

                break;
            case backwards3:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);

                if (currentTime > 0.4)  {

                    state = State.wait7;
                    time.reset();
                }

                break;
            case wait7:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);

                if (currentTime > 0.4)  {

                    state = State.check;
                    time.reset();
                }
                break;

            case check:
                if (colorSensor.red() >= 1 && colorSensor.blue() == 0){
                    state = state.secondstrafe;
                    time.reset();
                }
                else{
                    time.reset();
                    state = state.wait;
                }
            break;

            case wait:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);

                if (currentTime > 5.0)  {

                    state = State.hit;
                    time.reset();
                }
                break;


            case secondstrafe:
                BackMotor1.setPower(-0.5);
                FrontMotor1.setPower(0.5);
                BackMotor2.setPower(0.5);
                FrontMotor2.setPower(-0.5);
                if (currentTime > 1.0)  {
                    state = State.secondstrafe2;
                    time.reset();
                }

                break;
            case secondstrafe2:

                BackMotor1.setPower(-0.25);
                FrontMotor1.setPower(0.25);
                BackMotor2.setPower(0.25);
                FrontMotor2.setPower(-0.25);

                if (ods.getLightDetected() > 0.4) {

                    state = State.hit2;
                    time.reset();

                }
                break;
            case hit2:
                FrontMotor1.setPower(1.0);
                FrontMotor2.setPower(1.0);
                BackMotor1.setPower(1.0);
                BackMotor2.setPower(1.0);
                if (currentTime > 0.5)  {

                    state = State.backwards4;
                    time.reset();
                }

                break;
            case backwards4:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);

                if (currentTime > 0.4)  {

                    state = State.check2;
                    time.reset();
                }

                break;
            case check2:
                if (colorSensor.red() >= 1 && colorSensor.blue() == 0){
                    state = state.done;
                    time.reset();
                }
                else{
                    time.reset();
                    state = state.wait2;
                }
                break;
            case wait2:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
                if(currentTime > 5.0){
                    time.reset();
                    state = state.hit2;
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



