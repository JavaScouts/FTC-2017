package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


/**
 * Created by Liam on 11/20/2017.
 */
@Autonomous(name="AutoTest")
public class StrafeTest extends LinearOpMode {

    LZRobot robot = new LZRobot();

    public void runOpMode() {

        robot.init(hardwareMap, this);

        waitForStart();

        telemetry.addData("Left", "5IN");
        telemetry.update();

        robot.moveDist("left", 0.7, 5, 5);

        sleep(250);

        telemetry.addData("Right", "5IN");
        telemetry.update();

        robot.moveDist("right", 0.7, 5, 5);

        telemetry.addData("Forwards", "5IN");
        telemetry.update();

        robot.moveDist("forwards", 1.0, 5, 5);

    }

}
