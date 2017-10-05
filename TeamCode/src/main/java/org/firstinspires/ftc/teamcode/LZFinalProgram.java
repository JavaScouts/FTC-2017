package org.firstinspires.ftc.teamcode;

import android.graphics.LinearGradient;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@TeleOp(name = "Leon's Final Program")
public class LZFinalProgram extends LinearOpMode {

    LZRobot robot = new LZRobot();

    public void runOpMode() {

        robot.init(hardwareMap);
        telemetry.addData("Initialization", "Completed");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.x) {

                robot.Coll.setPower(1.0);

            } else if (gamepad2.b) {

                robot.Coll.setPower(0);

            }

            if (gamepad2.y) {

                robot.Arm.setPower(1.0);

            } else if (gamepad2.a) {

                robot.Arm.setPower(0);

            }


            float leftY = gamepad1.left_stick_y;
            float leftX = gamepad1.left_stick_x;
            float rightX = gamepad1.right_stick_x;

            float L = -leftY+leftX;

            float R = -leftY-leftX;

            float max=abs(L);

            if(max<abs(R)) max=abs(R);

            if(max>1){L/=max; R/=max;}

            robot.BackMotor1.setPower(L);
            robot.FrontMotor1.setPower(L);
            robot.BackMotor2.setPower(-leftY);
            robot.FrontMotor2.setPower(-leftY);
            robot.BackMotor2.setPower(R);
            robot.BackMotor2.setPower(R);
            robot.BackMotor1.setPower(-leftY);
            robot.FrontMotor1.setPower(-leftY);

            /*
            robot.move("left", -leftX);

            robot.move("right", leftX);
            */
            /*
            robot.rotate("clock", rightX);

            robot.rotate("cclock", -rightX);
            */

            }
        }
    }