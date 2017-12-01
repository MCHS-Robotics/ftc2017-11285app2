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

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Kaleb Auto ", group="Auto")
public class KalebQuickAuto extends LinearOpMode {
    DcMotor fl,fr,bl,br;
    final double Dr = 19;
    final double Cr = Math.PI * Dr;
    final double Dw = 4;
    final double Cw = Math.PI * Dw;
    final int Mstep = 1120;

    @Override
    public void runOpMode() {
        fl = hardwareMap.dcMotor.get("fl");
        fr = hardwareMap.dcMotor.get("fr");
        bl = hardwareMap.dcMotor.get("bl");
        br = hardwareMap.dcMotor.get("br");
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        moveForward(Cw/Math.sqrt(2));
        }


    public void moveForward(double inches){
        int pos = (int)(inches * Mstep * Math.sqrt(2) / Cw);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setTargetPosition(pos);
        fr.setTargetPosition(pos);
        bl.setTargetPosition(pos);
        br.setTargetPosition(pos);
        fl.setPower(0.3);
        fr.setPower(0.3);
        bl.setPower(0.3);
        br.setPower(0.3);
        while(fl.isBusy() || fr.isBusy() || bl.isBusy() || br.isBusy()){
           if(!fl.isBusy())fl.setPower(0);
            if(!fr.isBusy())fr.setPower(0);
            if(!bl.isBusy())bl.setPower(0);
            if(!br.isBusy())br.setPower(0);

        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveBackward(double inches){
        int pos = -(int)(inches * Mstep * Math.sqrt(2) / Cw);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setTargetPosition(pos);
        fl.setPower(0.3);
        fr.setPower(-0.3);
        bl.setPower(-0.3);
        br.setPower(-0.3);
        while(fl.isBusy()){
            telemetry.addData("Forward","Encoder: " + fl.getCurrentPosition());
            telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void moveLeft(double inches){
        int pos = -(int)((inches * Mstep * Math.sqrt(2) / Cw)*3/4);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setTargetPosition(pos);
        fl.setPower(0.3);
        fr.setPower(0.3);
        bl.setPower(0.3);
        br.setPower(-0.3);
        while(fl.isBusy()){
            telemetry.addData("Forward","Encoder: " + fl.getCurrentPosition());
            telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveRight(double inches){
        int pos = (int)(inches * Mstep * Math.sqrt(2) / Cw);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setTargetPosition(pos);
        fl.setPower(0.3);
        fr.setPower(-0.3);
        bl.setPower(-0.3);
        br.setPower(0.3);
        while(fl.isBusy()){
            telemetry.addData("Forward","Encoder: " + fl.getCurrentPosition());
            telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void clockwise(double deg){
        int pos = (int)(deg * Cr * Mstep / 360 / Cw);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setTargetPosition(pos);
        fl.setPower(0.3);
        fr.setPower(-0.3);
        bl.setPower(0.3);
        br.setPower(-0.3);
        while(fl.isBusy()){
            telemetry.addData("Forward","Encoder: " + fl.getCurrentPosition());
            telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void cClockwise(double deg){
        int pos = -(int)(deg * Cr * Mstep / 360 / Cw);
        fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fl.setTargetPosition(pos);
        fl.setPower(0.3);
        fr.setPower(0.3);
        bl.setPower(-0.3);
        br.setPower(0.3);
        while(fl.isBusy()){
            telemetry.addData("Forward","Encoder: " + fl.getCurrentPosition());
            telemetry.update();
        }
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
