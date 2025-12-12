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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Autonomous OpMode for Close Side Blue / Far Side Red starting position.
 * 
 * <p>
 * This autonomous routine is designed for matches where the robot starts on the
 * close side
 * of the field when playing on the blue alliance, or the far side when playing
 * on the red alliance.
 * The routine executes a timed movement sequence:
 * <ol>
 * <li>Waits 10 seconds after match start</li>
 * <li>Drives forward at 50% power for 1.25 seconds</li>
 * <li>Strafes left at 50% power for 0.5 seconds</li>
 * <li>Stops all movement and waits for the remainder of the match</li>
 * </ol>
 * 
 * <p>
 * This OpMode uses a mecanum drive train with four motors configured as:
 * <ul>
 * <li>Left Front (lf) - Reversed</li>
 * <li>Left Back (lb) - Reversed</li>
 * <li>Right Front (rf) - Forward</li>
 * <li>Right Back (rb) - Reversed</li>
 * </ul>
 * 
 * <p>
 * The drive calculations use a holonomic drive algorithm that combines axial
 * (forward/backward),
 * lateral (strafe left/right), and yaw (rotation) components. Power values are
 * normalized to ensure
 * no motor exceeds 100% power while maintaining the desired motion vector.
 * 
 * <p>
 * Note: This routine differs from FarSideBlueCloseSideRed by strafing left
 * instead of right,
 * which accounts for the different starting position on the field.
 * 
 * @see <a href="https://javadoc.io/doc/org.firstinspires.ftc">FTC SDK
 *      Documentation</a>
 * @see LinearOpMode
 * @see DcMotor
 */
@Autonomous(name = "CloseSideBlueFarSideRed", group = "Autonomous")
public class CloseBlue extends LinearOpMode {

    /** Timer to track elapsed time during autonomous execution */
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double leftFrontPower = 0;

    /** Maximum power value used for normalization */
    private double max;

    /** Axial movement component: forward (negative) and backward (positive) */
    private double axial = -0.5;

    /** Lateral movement component: strafe left (negative) and right (positive) */
    private double lateral = 0.0;

    /**
     * Yaw rotation component: counter-clockwise (positive) and clockwise (negative)
     */
    private double yaw = 0.0;

    private DcMotor catapult1 = null;
    private DcMotor catapult2 = null;

    private double CATAPULT_UP_POWER = -1.0;
    private double CATAPULT_DOWN_POWER = 1.0;
    private double CATAPULT_HOLD_POWER = 0.2;

    private enum CatapultModes {
        UP, DOWN, HOLD
    }

    private CatapultModes pivotMode;

    /**
     * Main execution method for the autonomous OpMode.
     * 
     * <p>
     * Initializes hardware, waits for start signal, then executes the autonomous
     * routine.
     * The routine consists of timed movements: forward drive, then left strafe,
     * then stop.
     */
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFrontDrive = hardwareMap.get(DcMotor.class, "lf");
        leftBackDrive = hardwareMap.get(DcMotor.class, "lb");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rf");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rb");

        catapult1 = hardwareMap.get(DcMotor.class, "catapult1");
        catapult2 = hardwareMap.get(DcMotor.class, "catapult2");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        catapult1.setDirection(DcMotor.Direction.REVERSE); // Backwards should pivot DOWN, or in the stowed position.
        catapult2.setDirection(DcMotor.Direction.FORWARD);

        catapult1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        catapult2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontPower = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            calculateCatapultPower(CatapultModes.DOWN);
            sleep(2000);
            calculateCatapultPower(CatapultModes.UP);
            sleep(10000-2000);

            calculateDriveCode();
            sleep(1250);

            axial = 0.0;
            lateral = -0.5;
            sleep(10);
            calculateDriveCode();
            sleep(500);

            lateral = 0.0;
            calculateDriveCode();

            sleep(30000);
        }
    }

    /**
     * Calculates and applies power to all four drive motors using holonomic drive
     * algorithm.
     * 
     * <p>
     * This method combines the axial (forward/backward), lateral (strafe), and yaw
     * (rotation)
     * components to determine individual wheel powers. The powers are normalized to
     * ensure no
     * motor exceeds 100% power while maintaining the desired motion vector.
     * 
     * <p>
     * The mecanum drive algorithm distributes power as follows:
     * <ul>
     * <li>Left Front: axial + lateral + yaw</li>
     * <li>Right Front: axial - lateral - yaw</li>
     * <li>Left Back: axial - lateral + yaw</li>
     * <li>Right Back: axial + lateral - yaw</li>
     * </ul>
     */
    public void calculateDriveCode() {
        leftFrontPower = axial + lateral + yaw;
        rightFrontPower = axial - lateral - yaw;
        leftBackPower = axial - lateral + yaw;
        rightBackPower = axial + lateral - yaw;

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
    }

    public void calculateCatapultPower(CatapultModes pivotMode) {
        if (pivotMode == CatapultModes.UP) {
            catapult1.setPower(CATAPULT_UP_POWER);
            catapult2.setPower(CATAPULT_UP_POWER);
        } else if (pivotMode == CatapultModes.DOWN) {
            catapult1.setPower(CATAPULT_DOWN_POWER);
            catapult2.setPower(CATAPULT_DOWN_POWER);
        } else if (pivotMode == CatapultModes.HOLD) {
            catapult1.setPower(CATAPULT_HOLD_POWER);
            catapult2.setPower(CATAPULT_HOLD_POWER);
        }

        telemetry.addData("Catapult1 Current/Target/power", "%d, %d, %4.2f",
                catapult1.getCurrentPosition(), catapult1.getTargetPosition(), catapult1.getPower());
        telemetry.addData("Catapult2 Current/Target/power", "%d, %d, %4.2f",
                catapult2.getCurrentPosition(), catapult2.getTargetPosition(), catapult2.getPower());
        telemetry.addData("Catapult MODE", "%s", pivotMode.toString());

    }
}
