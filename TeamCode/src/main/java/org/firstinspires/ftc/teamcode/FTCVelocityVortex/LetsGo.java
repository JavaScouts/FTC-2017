/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.FTCVelocityVortex;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


//@Autonomous(name="LetsGo", group="LetsGo")
public class LetsGo extends LinearOpMode {


    DcMotor left;
    DcMotor right;
    double leftPower, rightPower, correction;
    final double PERFECT_COLOR_VALUE = 0.23;
    final double SCALE_VALUE = .1;
    final double FOLLOW_POWER = 0.15;

    OpticalDistanceSensor lightSensor;



    @Override
    public void runOpMode() throws InterruptedException {

        left = hardwareMap.dcMotor.get("left");
        right = hardwareMap.dcMotor.get("right");

        lightSensor = hardwareMap.opticalDistanceSensor.get("sensor_ods");

        lightSensor.enableLed(true);

        telemetry.addData("Color Value", lightSensor.getLightDetected());
        waitForStart();

        while (opModeIsActive()) {
            // Get a correction
            correction = (PERFECT_COLOR_VALUE - lightSensor.getLightDetected()) * SCALE_VALUE;
            telemetry.addData("Color Value", lightSensor.getLightDetected());
            telemetry.addData("Correction", correction);
            telemetry.update();


            // Sets the powers so they are no less than .075 and apply to correction
            if (correction <= 0) {
                leftPower = FOLLOW_POWER - correction;
                rightPower = FOLLOW_POWER;
            } else {
                leftPower = FOLLOW_POWER;
                rightPower = FOLLOW_POWER + correction;
            }

            // Sets the powers to the motors
            left.setPower(leftPower);
            right.setPower(rightPower);
        }



    }
}
