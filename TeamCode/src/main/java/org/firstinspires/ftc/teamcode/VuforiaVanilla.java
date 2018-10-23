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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="Autono", group="Auto")
//@Disabled
public class VuforiaVanilla extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo servo1 = null;
    private DcMotor arm = null;
    private DcMotor arm2 = null;
    private Servo servo2 = null;
    //sup
    @Override
    public void runOpMode() {

        //todo : this piece of code into a sub-routine init_motors()
        /*leftDrive  = hardwareMap.get(DcMotor.class, "ld");
        rightDrive = hardwareMap.get(DcMotor.class, "rd");
        servo1 = hardwareMap.get(Servo.class, "servo1");
        arm = hardwareMap.get(DcMotor.class, "arm");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        arm2 = hardwareMap.get(DcMotor.class, "arm2");*/

        init_motors();

        //todo : this piece of code move into reset_motors()
        waitForStart();
        runtime.reset();
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //reset_motors(0);



        //todo : Confirm rotation number from motor documentation
        leftDrive.setTargetPosition(1400);
        rightDrive.setTargetPosition(1400);
        rightDrive.setPower(0.5);
        leftDrive.setPower(0.5);


        while(opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy()))
        {
                //leftDrive.setPower(50);
               // rightDrive.setPower(50);

        }
        //leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /*leftDrive.setTargetPosition(leftDrive.getCurrentPosition() - 500);
        rightDrive.setTargetPosition(rightDrive.getCurrentPosition() + 500);*/

        leftDrive.setTargetPosition(1400);
        rightDrive.setTargetPosition(-1400);
        rightDrive.setPower(0.5);
        leftDrive.setPower(0.5);


        //rightDrive.setPower(100);
        while(opModeIsActive() && (leftDrive.isBusy() || rightDrive.isBusy()))
        {
           // rightDrive.setPower(50);
           // leftDrive.setPower(50);

        }
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setTargetPosition(1400);
        rightDrive.setTargetPosition(-1400);
        rightDrive.setPower(0.5);
        leftDrive.setPower(0.5);




        }

        public void init_motors() {
            leftDrive  = hardwareMap.get(DcMotor.class, "ld");
            rightDrive = hardwareMap.get(DcMotor.class, "rd");
            servo1 = hardwareMap.get(Servo.class, "servo1");
            arm = hardwareMap.get(DcMotor.class, "arm");
            servo2 = hardwareMap.get(Servo.class, "servo2");
            arm2 = hardwareMap.get(DcMotor.class, "arm2");
        }


        public void reset_motors(int power){

            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setPower(power);
            rightDrive.setPower(power);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }


        public void tank_move(int inches, double power){
            double circumference = 12.56;
            double rotations = inches/circumference;
            double encoder_count = rotations*1400;
            int encoder_count1 = (int) encoder_count;

            leftDrive.setTargetPosition(encoder_count1);
            rightDrive.setTargetPosition(encoder_count1);
            rightDrive.setPower(power);
            leftDrive.setPower(power);
        }
    }

