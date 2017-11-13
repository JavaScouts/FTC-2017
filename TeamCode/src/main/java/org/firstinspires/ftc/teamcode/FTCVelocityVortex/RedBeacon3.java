package org.firstinspires.ftc.teamcode.FTCVelocityVortex;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "RedBeacon3")


public class RedBeacon3 extends OpMode {
    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor Arm;
    DcMotor coll;
    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor ods;
    ModernRoboticsI2cGyro gyro;
    //double initZ;
    double input;
    private double prevRangeValue = 0.0;

    ElapsedTime time;



    enum State {Init, Straight1, backwar, forward, StrafeLife, forward2, forward3, Check, RedHit, StrafeLeft, BlueHit, backwards, bluebackwards, StrafeRight, Check2, fshoot, wait7, Shoot, Collector, Shoot2, backbeacon, StrafeBeacon, StrafeBeacon2, wait3, forward5, Check3, RedHit2, StrafeLeft2, BlueHit2, backwards2, bluebackwards2, StrafeRight2, wait5, Check4, done }

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
        gyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");

        FrontMotor1.setDirection(DcMotor.Direction.FORWARD);
        FrontMotor2.setDirection(DcMotor.Direction.REVERSE);
        BackMotor1.setDirection(DcMotor.Direction.FORWARD);
        BackMotor2.setDirection(DcMotor.Direction.REVERSE);
        colorSensor.enableLed(false);
        rangeSensor.enableLed(false);
        ods.enableLed(true);


    }

    @Override
    public void start() {

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
        //initZ = gyro.getIntegratedZValue();
        double currentTime = time.time();
        ods.enableLed(true);
        //telemetry.addData("gyro", initZ);

        input = rangeSensor.getDistance(DistanceUnit.CM);
        if (input > 100.0 && prevRangeValue <= 100.0)
        {
            input = prevRangeValue;
        }
        else
        {
            prevRangeValue = input;
        }
        telemetry.update();


        switch (state) {
            case Init:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
                telemetry.addData(">", "Gyro Calibrating. Do Not move!");
                telemetry.update();
                gyro.calibrate();

                if (currentTime > 0.2) {
                    time.reset();

                    state = State.Straight1;

                }
                break;
            case Straight1:
                FrontMotor1.setPower(0.5);
                FrontMotor2.setPower(0.5);
                BackMotor1.setPower(0.5);
                BackMotor2.setPower(0.5);

                if (currentTime > 0.75) {

                    state = State.backwar;
                    time.reset();
                }

                break;
            case backwar:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0.5);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0.5);

                if (currentTime > 1.65) {

                    state = State.forward;
                    time.reset();
                }


                break;
            case forward:
                if (rangeSensor.getDistance(DistanceUnit.CM) > 45) {
                    FrontMotor1.setPower(0.2);
                    FrontMotor2.setPower(0.2);
                    BackMotor1.setPower(0.2);
                    BackMotor2.setPower(0.2);
                } else {
                    state = State.StrafeLife;
                    time.reset();
                }
                break;
            case StrafeLife:
                FrontMotor1.setPower(0.25);
                FrontMotor2.setPower(-0.25);
                BackMotor1.setPower(-0.25);
                BackMotor2.setPower(0.25);
                if (ods.getLightDetected() > 0.5) {
                    state = State.forward2;
                    time.reset();
                }
                break;
            case forward2:
                FrontMotor1.setPower(0.0);
                FrontMotor2.setPower(0.0);
                BackMotor1.setPower(0.0);
                BackMotor2.setPower(0.0);
                if (currentTime > 1.0) {
                    state = State.forward3;
                    time.reset();
                }
                break;
            case forward3:

                FrontMotor1.setPower(0.2);
                FrontMotor2.setPower(0.2);
                BackMotor1.setPower(0.2);
                BackMotor2.setPower(0.2);
                if (currentTime > 0.12) {
                    state = State.Check;
                    time.reset();
                }
                break;


            case Check:
                if (colorSensor.blue() > colorSensor.red()) {
                    state = State.RedHit;
                    time.reset();
                }
                if (colorSensor.red() > colorSensor.blue()) {
                    state = State.StrafeLeft;
                    time.reset();
                }
                break;

            case RedHit:
                FrontMotor1.setPower(0.25);
                FrontMotor2.setPower(0.25);
                BackMotor1.setPower(0.25);
                BackMotor2.setPower(0.25);
                if (currentTime > 1.0) {
                    state = State.backwards;
                    time.reset();
                }
                break;

            case StrafeLeft:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(0.5);
                BackMotor1.setPower(0.5);
                BackMotor2.setPower(-0.5);
                if (currentTime > 0.47) {
                    state = State.BlueHit;
                    time.reset();
                }
                break;

            case BlueHit:
                FrontMotor1.setPower(0.5);
                FrontMotor2.setPower(0.5);
                BackMotor1.setPower(0.5);
                BackMotor2.setPower(0.5);
                if (currentTime > 1.0) {
                    state = State.bluebackwards;
                    time.reset();
                }
                break;

            case backwards:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);
                if (currentTime > 0.37) {
                    state = State.Check2;
                    time.reset();
                }
                break;

            case bluebackwards:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);
                if (currentTime > 0.5) {
                    state = State.StrafeRight;
                    time.reset();
                }
                break;

            case StrafeRight:
                FrontMotor1.setPower(0.25);
                FrontMotor2.setPower(-0.25);
                BackMotor1.setPower(-0.25);
                BackMotor2.setPower(0.25);
                if (ods.getLightDetected() > 0.5) {
                    state = State.Check2;
                    time.reset();
                }
                break;

            case Check2:
                if (colorSensor.blue() == 0) {
                    state = State.fshoot;
                    time.reset();
                }
                if (colorSensor.blue() > colorSensor.red()) {
                    state = State.RedHit;
                    time.reset();
                }
                if (colorSensor.red() > colorSensor.blue() && colorSensor.blue() > 0) {
                    state = State.StrafeLeft;
                    time.reset();
                }
                break;

            case fshoot:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);
                if (currentTime > 0.3) {
                    state = State.wait7;
                    time.reset();
                }
                break;
            case wait7:
                FrontMotor1.setPower(0.0);
                FrontMotor2.setPower(0.0);
                BackMotor1.setPower(0.0);
                BackMotor2.setPower(0.0);
                if (currentTime > 1.5) {
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

                if (currentTime > 0.5) {
                    state = State.Collector;
                    time.reset();
                }
                break;

            case Collector:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
                Arm.setPower(0);
                coll.setPower(1.0);

                if (currentTime > 1.0) {
                    state = State.Shoot2;
                    time.reset();
                }
                break;

            case Shoot2:

                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
                Arm.setPower(1.0);
                coll.setPower(0);

                if (currentTime > 0.5) {
                    state = State.backbeacon;
                    time.reset();
                }
                break;

            case backbeacon:
                if(rangeSensor.getDistance(DistanceUnit.CM) > 45){
                FrontMotor1.setPower(0.2);
                FrontMotor2.setPower(0.2);
                BackMotor1.setPower(0.2);
                BackMotor2.setPower(0.2);
                Arm.setPower(0);
                coll.setPower(0);
                 }else{
                    state = State.StrafeBeacon;
                    time.reset();
                }
                break;

            case StrafeBeacon:
                FrontMotor1.setPower(0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(0.5);
                if (currentTime > 1.0){
                    state = State.StrafeBeacon2;
                    time.reset();
                }
                break;

            case StrafeBeacon2:
                FrontMotor1.setPower(0.25);
                FrontMotor2.setPower(-0.25);
                BackMotor1.setPower(-0.25);
                BackMotor2.setPower(0.25);
                if (ods.getLightDetected() > 0.5){
                    state = State.wait3;
                    time.reset();
                }
                break;
            case wait3:
                FrontMotor1.setPower(0.0);
                FrontMotor2.setPower(0.0);
                BackMotor1.setPower(0.0);
                BackMotor2.setPower(0.0);
                if (currentTime > 1.0) {
                    state = State.forward5;
                    time.reset();
                }
                break;
            case forward5:
                FrontMotor1.setPower(0.2);
                FrontMotor2.setPower(0.2);
                BackMotor1.setPower(0.2);
                BackMotor2.setPower(0.2);
                if (currentTime > 0.24) {
                    state = State.Check3;
                    time.reset();
                }
                break;

            case Check3:
                if(colorSensor.blue() > colorSensor.red()){
                    state = State.RedHit2;
                    time.reset();
                }
                if (colorSensor.red() > colorSensor.blue()){
                    state = State.StrafeLeft2;
                    time.reset();
                }
                break;

            case RedHit2:
                FrontMotor1.setPower(0.5);
                FrontMotor2.setPower(0.5);
                BackMotor1.setPower(0.5);
                BackMotor2.setPower(0.5);
                if (currentTime > 1.0){
                    state = State.backwards2;
                    time.reset();
                }
                break;

            case backwards2:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);
                if (currentTime > 0.5){
                    state = State.wait5;
                    time.reset();
                }
                break;

            case StrafeLeft2:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(0.5);
                BackMotor1.setPower(0.5);
                BackMotor2.setPower(-0.5);
                if (currentTime > 0.47){
                    state = State.BlueHit2;
                    time.reset();
                }
                break;

            case BlueHit2:
                FrontMotor1.setPower(0.5);
                FrontMotor2.setPower(0.5);
                BackMotor1.setPower(0.5);
                BackMotor2.setPower(0.5);
                if (currentTime > 1.0){
                    state = State.bluebackwards2;
                    time.reset();
                }
                break;

            case bluebackwards2:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);
                if (currentTime > 0.5){
                    state = State.StrafeRight2;
                    time.reset();
                }
                break;


            case StrafeRight2:
                FrontMotor1.setPower(0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(0.5);
                if (ods.getLightDetected() > 0.5){
                    state = State.wait5;
                    time.reset();
                }
                break;
            case wait5:
                FrontMotor1.setPower(0.0);
                FrontMotor2.setPower(0.0);
                BackMotor1.setPower(0.0);
                BackMotor2.setPower(0.0);
                if (currentTime > 1.0) {
                    state = State.Check4;
                    time.reset();
                }
                break;

            case Check4:
                if (colorSensor.blue() == 0){
                    state = State.done;
                    time.reset();
                }
                if(colorSensor.blue() > colorSensor.red()){
                    state = State.RedHit2;
                    time.reset();
                }
                if (colorSensor.red() > colorSensor.blue() && colorSensor.blue() > 0){
                    state = State.StrafeLeft2;
                    time.reset();
                }
                break;

            case done:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
                Arm.setPower(0);
                coll.setPower(0);








        }
    }
}