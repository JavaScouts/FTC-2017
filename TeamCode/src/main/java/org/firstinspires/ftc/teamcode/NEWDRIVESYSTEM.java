package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Math.abs;

@TeleOp(name = "NewDriveSys")
public class NEWDRIVESYSTEM extends LinearOpMode {

    LZRobot robot = new LZRobot();

    public void runOpMode() {

        robot.init(hardwareMap, this);

        telemetry.addData("Initialization", "Completed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            robot.manualDrive();

            robot.moveRobot();

            telemetry.update();
        }
    }
}