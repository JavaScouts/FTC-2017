package org.firstinspires.ftc.teamcode;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@TeleOp(name = "LZFinalTestProgrm")
public class LZFinalTestProgram extends LinearOpMode {

    LZTestChassis robot = new LZTestChassis();

    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Initialization", "Completed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad2.x) {
                robot.servo1.setPosition(0.64);
                robot.servo2.setPosition(0.34);
            } else {
                robot.servo1.setPosition(0);
                robot.servo2.setPosition(0.80);
            }

                //           telemetry.addData("servopos", robot.servo1.getPosition());
                //         telemetry.addData("servo2pos", robot.servo2.getPosition());
                //       telemetry.update();

            float leftY = -gamepad1.left_stick_y;
            float leftX = -gamepad1.left_stick_x;
            float rightX = gamepad1.right_stick_x;
            float quck = -gamepad2.left_stick_y;

            robot.mecDrive(leftY, leftX, rightX, 0.05);
            robot.Arm.setPower(quck);

        }
    }
}