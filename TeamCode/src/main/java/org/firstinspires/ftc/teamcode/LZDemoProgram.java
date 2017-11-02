package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Liam on 10/27/2017.
 */

public class LZDemoProgram extends LinearOpMode {

    LZRobot robot = new LZRobot();

    public void runOpMode() {

        robot.init(hardwareMap, this);

        waitForStart();

        while(opModeIsActive()) {

            float leftY = -gamepad1.left_stick_y;
            float rightY = -gamepad1.right_stick_y;

            robot.leftDrive.setPower(leftY);
            robot.backLDrive.setPower(leftY);
            robot.rightDrive.setPower(rightY);
            robot.backRDrive.setPower(rightY);

        }

    }

}
