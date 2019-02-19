/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import java.util.spi.TimeZoneNameProvider;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;

public class RunMotorArm extends Command {
  public RunMotorArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm); 
    requires(Robot.clock); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.clock.resetClock();
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
   SmartDashboard.putNumber("Enc position", Robot.arm.getSensorValue()); 

   double speed = OI.controllerOne.getRawAxis(1) * OI.controllerOne.getRawAxis(1); 
   double runningSpeed = speed / 2; 
   SmartDashboard.putNumber("Arm speed", runningSpeed); 
   System.out.println(Robot.arm.getSensorValue());

     //JUST MOTION CONTROLLER CODE
   /* if (OI.controllerOne.getRawAxis(1) > .1 && !OI.armPosZero.get() && !OI.armPosOne.get() && !OI.armPosTwo.get() && !OI.armPosThree.get()) {
      
    } else if (OI.controllerOne.getRawAxis(1) < 0 && !OI.armPosZero.get() && !OI.armPosOne.get() && !OI.armPosTwo.get() && !OI.armPosThree.get()) {  
      Robot.arm.runArm(-runningSpeed);
    }
    else if (OI.controllerOne.getRawAxis(1) == 0 && !OI.armPosZero.get() && !OI.armPosOne.get() && !OI.armPosTwo.get() && !OI.armPosThree.get()) {
      Robot.arm.runArm(0);
    } */

    //ALTERING PID CODE 
    if (OI.controllerOne.getRawAxis(1) > 0) {
      double currentPIDLocation = Robot.arm.getSensorValue(); 
      double location = OI.controllerOne.getRawAxis(1); 
    }

    if (OI.armPosZero.get()) {
     System.out.println("B button pressed"); 
     // if(Robot.arm.armMax/2 >= Robot.arm.getSensorValue() && Robot.arm.armMin/2 < Robot.arm.getSensorValue()) {
        Robot.arm.runPIDArm(Robot.arm.posZero);
   //     System.out.print(Robot.arm.runPIDArm(Robot.arm.armMax)); 
        SmartDashboard.putNumber("Target position", Robot.arm.posZero); 
     // }
    } else if (OI.armPosOne.get()) {
      //Robot.arm.runPIDArm(Robot.arm.posOne);
      //SmartDashboard.putNumber("Target position", Robot.arm.posOne); 
    } else if (OI.armPosTwo.get()) {
      //Robot.arm.runPIDArm(Robot.arm.posTwo);
      //SmartDashboard.putNumber("Target position", Robot.arm.posTwo); 
    } else if (OI.armPosThree.get()) {
      //Robot.arm.runPIDArm(Robot.arm.posThree); 
      //SmartDashboard.putNumber("Target position", Robot.arm.posThree); 
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
