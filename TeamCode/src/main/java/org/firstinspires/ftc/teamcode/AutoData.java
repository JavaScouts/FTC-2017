package org.firstinspires.ftc.teamcode;

import android.text.ParcelableSpan;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Liam on 11/14/2017.
 */

public class AutoData extends LinearOpMode {

    LZRobot robot = new LZRobot();
    LZNavSys nav = new LZNavSys();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, this);
        nav.initVuforia(this, robot);
        telemetry.addData("Initialiation", "Complete");
        telemetry.update();

        robot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.getCurrentPosition();
        telemetry.addData("Path",  "Starting at %7d : %7d : %7d : %7d",
                robot.getCurrentPosition(1),
                robot.getCurrentPosition(3),
                robot.getCurrentPosition(2),
                robot.getCurrentPosition(4));
        telemetry.update();

        waitForStart();

        nav.activateTracking();

        telemetry.addData("ColorSensorRed", robot.color.red());
        telemetry.addData("ColorSensorBlue", robot.color.blue());

        telemetry.addData("Relic:", nav.whatRelic());

    }
}
