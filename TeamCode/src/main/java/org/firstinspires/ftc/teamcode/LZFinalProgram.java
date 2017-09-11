package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Leon's Final Program")
public class LZFinalProgram extends OpMode {

    LZRobot robot = new LZRobot();

    public void init(){

        robot.init(hardwareMap);
        telemetry.addData("Initialization", "Completed");
        telemetry.update();

    }


    public void loop() {

        if(gamepad2.x) {

            robot.Coll.setPower(1.0);

        }

        else if(gamepad2.b){

            robot.Coll.setPower(0);

        }

        if(gamepad2.y) {

            robot.Arm.setPower(1.0);

        }
        else if(gamepad2.a) {

            robot.Arm.setPower(0);

        }




        double leftY = -gamepad1.left_stick_y;
        double leftX = gamepad1.left_stick_x;
        double rightX = gamepad1.right_stick_x;

        if(leftY > 0) {

            robot.move("forwards", leftY);

        } else {

            robot.move("backwards", -leftY);

        }

        if(leftX > 0) {

            robot.move("left", -leftX);

        } else {

            robot.move("right", leftX);

        }

        if(rightX > 0) {

            robot.rotate("clock", rightX);

        } else {

            robot.rotate("cclock", -rightX);

        }

    }

}
