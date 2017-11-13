package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.abs;

@TeleOp(name = "Final(NoTracking)")
public class FinalProgramNoTracking extends LinearOpMode {

    LZRobot robot = new LZRobot();
    LZNavSys nav = new LZNavSys();

    public void runOpMode() {

        robot.init(hardwareMap, this);
        nav.initVuforia(this, robot);

        telemetry.addData("Initialization", "Completed");
        telemetry.update();

        waitForStart();

        nav.activateTracking();

        while (opModeIsActive()) {

            telemetry.addData("Relic:", nav.whatRelic());

            robot.slide.setPower(-gamepad2.left_stick_y);

            if (gamepad2.a) {

                robot.s1.setPosition(1.0);

            } else if (gamepad2.b) {

                robot.s1.setPosition(0.3);

            }

            robot.manualDrive();

            robot.moveRobot();

            robot.dpadDrive(1.0);

            telemetry.update();

        }
    }

}