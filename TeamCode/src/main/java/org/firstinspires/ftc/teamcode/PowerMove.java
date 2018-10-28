package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Autonomo", group="Auto")
public class PowerMove extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private Servo servo1 = null;
    private DcMotor arm = null;
    private DcMotor arm2 = null;
    private Servo servo2 = null;

    @Override
    public void runOpMode() {
        waitForStart();

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        rightDrive.setPower(1);
        leftDrive.setPower(1);
        sleep(100);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(100);
        leftDrive.setPower(1);
        rightDrive.setPower(0);
        sleep(100);
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        sleep(100);
        rightDrive.setPower(1);
        leftDrive.setPower(1);


    }

}
