package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "NewNEWDriveSys")
public class newNEWDRIVESYSTEM extends LinearOpMode {

    LZRobot robot = new LZRobot();

    public void runOpMode() {

        robot.init(hardwareMap, this);

        telemetry.addData("Initialization", "Completed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            robot.newManualDrive();

        }
    }
}