package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by seed on 9/12/17.
 */

public class LZAuto extends LinearOpMode {

    /* Declare OpMode members. */
    EncodedLZRobot robot   = new EncodedLZRobot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        //first we'll attempt to use time

        robot.init(hardwareMap);
        telemetry.addData("Initialization", "Complete");
        telemetry.update();

        waitForStart();

        robot.moveTime("forwards", 0.5, 0.75);
        robot.moveTime("backwards", 0.5, 1.65);


    }

}