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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.HardwareClasses.Controller;
import org.firstinspires.ftc.teamcode.HardwareClasses.Robot;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static org.firstinspires.ftc.utilities.Utils.hardwareMap;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Concept: NullOp", group = "Concept")
//Disabled
public class ConceptNullOp extends OpMode {

  DcMotor frontLeft, frontRight, backLeft, backRight, intakeFront, intakeBack;
  CRServo bottomRoller;
  Controller driver;

  @Override
  public void init() {
    frontLeft = hardwareMap.get(DcMotor.class, "front_left");
    frontRight = hardwareMap.get(DcMotor.class, "front_right");
    backLeft = hardwareMap.get(DcMotor.class, "back_left");
    backRight = hardwareMap.get(DcMotor.class, "back_right");
    intakeFront = hardwareMap.get(DcMotor.class, "intakefront");
    intakeBack = hardwareMap.get(DcMotor.class, "intakeback");
    bottomRoller = hardwareMap.get(CRServo.class, "bottomroller");
  
    driver = new Controller(gamepad1);
  
    frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
    frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
    intakeBack.setDirection(DcMotorSimple.Direction.REVERSE);
    bottomRoller.setDirection(DcMotorSimple.Direction.REVERSE);
  
    frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
  
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    driver.update();
    
    double drive = gamepad1.right_stick_y * -1;
    double strafe = gamepad1.right_stick_x * -1;
    double turn = gamepad1.left_stick_x * -1;
    
    if(gamepad1.right_trigger > .4) {
       drive /= 2.5;
       strafe /= 2.5;
       turn /= 2.5;
    }
  
    drive = Range.clip(drive, -1, 1);
    strafe = Range.clip(strafe, -1, 1);
    turn = Range.clip(turn, -1, 1);
  
    double flPower = drive - strafe - turn;
    double frPower = drive + strafe + turn;
    double blPower = drive + strafe - turn;
    double brPower = drive - strafe + turn;
  
    double maxPower = abs(max(max(abs(flPower), abs(frPower)), max(abs(blPower), abs(brPower))));
    if(maxPower > 1) { frPower /= maxPower; flPower /= maxPower; blPower /= maxPower; brPower /= maxPower; }
    else if(maxPower < .05 && maxPower > -.05) { flPower = 0; frPower = 0; blPower = 0; brPower = 0; }
  
    frontLeft.setPower(flPower);
    frontRight.setPower(frPower);
    backLeft.setPower(blPower);
    backRight.setPower(brPower);
    
    if(driver.cross.toggle()){
      intakeBack.setPower(1);
      intakeBack.setPower(1);
      bottomRoller.setPower(1);
    }else{
      intakeBack.setPower(0);
      intakeBack.setPower(0);
      bottomRoller.setPower(0);
    }
    
  }
}
