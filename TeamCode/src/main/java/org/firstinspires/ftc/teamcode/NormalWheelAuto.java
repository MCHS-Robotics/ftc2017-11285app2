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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Autonomous(name="Normal Wheel Auto", group="Auto")
public class NormalWheelAuto extends LinearOpMode {
    DcMotor L,R;
    final double Dr = 19;
    final double Cr = Math.PI * Dr;
    final double Dw = 4;
    final double Cw = Math.PI * Dw;
    final int Mstep = 1120;

    @Override
    public void runOpMode() {
        L = hardwareMap.dcMotor.get("l");
        R = hardwareMap.dcMotor.get("r");
        R.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        moveForward(12);
        moveBackward(12);
        cClockwise(90);
        clockwise(90);
        }


    public void moveForward(double inches){
        int pos = (int)(inches * Mstep / Cw);
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L.setTargetPosition(pos);
        R.setTargetPosition(pos);
        L.setPower(0.3);
        R.setPower(0.3);
        while(L.isBusy()&&R.isBusy()) {
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveBackward(double inches){
        int pos = -(int)(inches * Mstep / Cw);
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L.setTargetPosition(pos);
        R.setTargetPosition(pos);
        L.setPower(0.3);
        R.setPower(0.3);
        while(L.isBusy()&&R.isBusy()) {
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    public void clockwise(double deg){
        int pos = (int)(deg * Cr * Math.sqrt(2) * Mstep / 360 / Cw);
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L.setTargetPosition(pos);
        R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setTargetPosition(-pos);
        L.setPower(0.3);
        R.setPower(0.3);
        while(L.isBusy() && R.isBusy()){
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void cClockwise(double deg){
        int pos = (int)(deg * Cr * Math.sqrt(2) * Mstep / 360 / Cw);
        L.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        L.setTargetPosition(-pos);
        R.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        R.setTargetPosition(pos);
        L.setPower(0.3);
        R.setPower(0.3);
        while(L.isBusy() && R.isBusy()){
        }
        L.setPower(0);
        R.setPower(0);
        L.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        R.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
