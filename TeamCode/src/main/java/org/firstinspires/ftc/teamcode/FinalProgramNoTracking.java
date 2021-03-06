package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static java.lang.Math.abs;

@TeleOp(name = "Final(NoTracking)")
public class FinalProgramNoTracking extends LinearOpMode {

    LZRobot robot = new LZRobot();
    //LZNavSys nav = new LZNavSys();

    public void runOpMode() {

        robot.init(hardwareMap, this);
       // nav.initVuforia(this, robot);

        telemetry.addData("Initialization", "Completed");
        telemetry.update();

        waitForStart();

      //  nav.activateTracking();

        while (opModeIsActive()) {

         //   telemetry.addData("Relic:", nav.whatRelic());

            if (gamepad1.a){
                robot.s2.setPosition(0);
            } else if (gamepad1.b){
                robot.s2.setPosition(0.7);
            }

            robot.slide.setPower(gamepad2.left_stick_y * .5);
            robot.Arm.setPower(-gamepad2.right_stick_y * 0.35);

            robot.manualDrive();

            robot.moveRobot();

            robot.dpadDrive(0.75);

            telemetry.addData("Blue", robot.color.blue());
            telemetry.addData("Red", robot.color.red());
            telemetry.addData("Range", robot.range1.getDistance(DistanceUnit.CM));

            telemetry.update();

        }
    }
}