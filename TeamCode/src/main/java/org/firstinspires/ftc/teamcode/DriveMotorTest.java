package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class DriveMotorTest extends LinearOpMode {


    @Override
    public void runOpMode(){

        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "fl");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "fr");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "bl");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "br");
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                fl.setPower(1);
            }else if(gamepad1.b){
                fr.setPower(1);
            }else if(gamepad1.x){
                bl.setPower(1);
            } else if (gamepad1.y) {
                br.setPower(1);
            }
        }

    }
}