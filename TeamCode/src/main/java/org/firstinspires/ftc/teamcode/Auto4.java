package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name = "Shoot")

public class Auto4 extends OpMode {
    DcMotor FrontMotor1;
    DcMotor FrontMotor2;
    DcMotor BackMotor1;
    DcMotor BackMotor2;
    DcMotor Arm;
    DcMotor coll;
    ColorSensor colorSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;
    OpticalDistanceSensor ods;


    ElapsedTime time;


    enum State {Init, Straight1, Shoot, wCollect, wshoot, done}

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
        telemetry.addData("ColorSensorRed", colorSensor.red());
        telemetry.addData("ColorSensorBlue", colorSensor.blue());
        telemetry.addData("Range", rangeSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("ODS", ods.getLightDetected());
        double currentTime = time.time();

        switch (state) {
            case Init:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);

                if (currentTime > 10.0) {
                    time.reset();

                    state = State.Straight1;

                }
                break;
            case Straight1:
                FrontMotor1.setPower(-0.5);
                FrontMotor2.setPower(-0.5);
                BackMotor1.setPower(-0.5);
                BackMotor2.setPower(-0.5);

                if (currentTime > 1.2) {

                    state = State.Shoot;
                    time.reset();
                }

                break;
            case Shoot:
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);

                Arm.setPower(-0.95);

                if (currentTime > 0.5) {
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

                if (currentTime > 3.0) {
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

                Arm.setPower(-0.95);

                if (currentTime > 0.5) {
                    Arm.setPower(0);

                    state = State.done;
                    time.reset();
                }
                break;

            case done:
                Arm.setPower(0);
                FrontMotor1.setPower(0);
                FrontMotor2.setPower(0);
                BackMotor1.setPower(0);
                BackMotor2.setPower(0);
        }
    }
}



