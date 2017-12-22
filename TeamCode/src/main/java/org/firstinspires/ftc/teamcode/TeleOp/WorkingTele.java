/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotDrive.MoveableRobot;
import org.firstinspires.ftc.teamcode.RobotDrive.XOmniDrive;


@TeleOp(name="working tele", group="tele op")
//@Disabled
public class WorkingTele extends LinearOpMode {
    MoveableRobot robot;
    DcMotor liftP;
    Servo liftL,liftR;
    final float[] posL = {.0f,.32f},posR = {1,.6f},posJ = {0,.47f};
    boolean stateC = false,dir = false;

    /**
     * Runs a basic tele-op w/ movement:
     *  Left joystick: translational movement
     *  Right joystick: rotational movement
     *  Left bumper: close/open lift clamp
     *  B: raises lift
     *  A: lowers lift
     */
    @Override
    public void runOpMode() {
        liftP = hardwareMap.dcMotor.get("liftM");
        liftL = hardwareMap.servo.get("liftL");
        liftR = hardwareMap.servo.get("liftR");
        //jewel = hardwareMap.servo.get("jewel");
        liftP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftL.setPosition(posL[1]);
        liftR.setPosition(posR[1]);
        //jewel.setPosition(posJ[0]);

        robot = new XOmniDrive(hardwareMap);

        telemetry.addData("setup","initialized");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            robot.run(gamepad1, gamepad2);
            if (!stateC && gamepad2.left_bumper) {
                stateC = true;
                if (!dir) {
                    liftL.setPosition(posL[1]);
                    liftR.setPosition(posR[1]);
                    // jewel.setPosition(posJ[1]);
                    dir = true;
                } else {
                    liftL.setPosition(posL[0]);
                    liftR.setPosition(posR[0]);
                    // jewel.setPosition(posJ[0]);
                    dir = false;
                }
            }
            if (stateC && !gamepad2.left_bumper) {
                stateC = false;
            }

            if (gamepad2.b) {
                liftP.setPower(.2);
            } else if (gamepad2.a){
                liftP.setPower(-.2);
        }else{
                liftP.setPower(0);
            }
/*
            if(gamepad2.a){
                liftP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }else{
                liftP.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
*/
            idle();
        }
    }
}
