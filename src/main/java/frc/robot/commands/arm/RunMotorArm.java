/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.OI;
import frc.robot.Robot;

public class RunMotorArm extends Command {
  public boolean hold = false; 
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
   // Robot.clock.stopClock();     
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
  /*if (OI.controllerOne.getRawAxis(1) > .1 && !OI.armPosZero.get() && !OI.armPosOne.get() && !OI.armPosTwo.get() && !OI.armPosThree.get()) {
      
    } else if (OI.controllerOne.getRawAxis(1) < 0 && !OI.armPosZero.get() && !OI.armPosOne.get() && !OI.armPosTwo.get() && !OI.armPosThree.get()) {  
      Robot.arm.runArm(-runningSpeed);
    }
    else if (OI.controllerOne.getRawAxis(1) == 0 && !OI.armPosZero.get() && !OI.armPosOne.get() && !OI.armPosTwo.get() && !OI.armPosThree.get()) {
      Robot.arm.runArm(0);
    } */

    //ALTERING PID CODE 
    /*if (OI.controllerOne.getRawAxis(1) > .1  ) {
      Robot.clock.startClock(); 
      double actualPos = SmartDashboard.getNumber("Enc position", 0); 
      if(Robot.clock.getClock() > 1) {
        double currentPos = actualPos + 10; 
        Robot.arm.runPIDArm(currentPos);
        SmartDashboard.putNumber("Target position", currentPos); 
        Robot.clock.resetClock(); 
      }
    } */

    if (!OI.armPosZero.get() && !OI.armPosOne.get() && !OI.armPosTwo.get() && !OI.armPosThree.get() && !(OI.controllerOne.getPOV() == 90)) {
      double holdPos = SmartDashboard.getNumber("armTalon Motor Output Percent", 0); 
      Robot.arm.runArm(holdPos); 
    } else if ( !(Math.abs(OI.controllerOne.getRawAxis(1)) > .1) && hold == true) {
      Robot.arm.runArm(0); 
      hold = false; 
    }
    if (Math.abs(OI.controllerOne.getRawAxis(1)) > .1) {
      double armSpeedRun = OI.controllerOne.getRawAxis(1) / 3; 
      Robot.arm.runArm(armSpeedRun); 
      SmartDashboard.putNumber("Arm Value", armSpeedRun); 
    }
  
    if (OI.armPosZero.get()) {
     System.out.println("B button pressed"); 
     // if(Robot.arm.armMax/2 >= Robot.arm.getSensorValue() && Robot.arm.armMin/2 < Robot.arm.getSensorValue()) {
        Robot.arm.armTalon.selectProfileSlot(1, 0);
        Robot.arm.runPIDArm(Robot.arm.posZero);
   //     System.out.print(Robot.arm.runPIDArm(Robot.arm.armMax)); 
        SmartDashboard.putNumber("Target position", Robot.arm.posZero); 
        SmartDashboard.putNumber("AT MOP:", Robot.arm.armTalon.getMotorOutputPercent()); 
   
      } else if (OI.armPosOne.get()) {
      Robot.arm.armTalon.selectProfileSlot(1, 0);
      Robot.arm.runPIDArm(Robot.arm.posOne);
      SmartDashboard.putNumber("Target position", Robot.arm.posOne); 
    } else if (OI.armPosTwo.get()) {
      Robot.arm.armTalon.selectProfileSlot(1, 0);
      Robot.arm.runPIDArm(Robot.arm.posTwo);
      SmartDashboard.putNumber("Target position", Robot.arm.posTwo); 
    } else if (OI.armPosThree.get()) {
      Robot.arm.armTalon.selectProfileSlot(0, 0);
      Robot.arm.runPIDArm(Robot.arm.posThree); 
      SmartDashboard.putNumber("Target position", Robot.arm.posThree); 
    } else if (OI.controllerOne.getPOV() == 90) {
      Robot.arm.runPIDArm(Robot.arm.posAbsZero); 
      hold = true; 
      SmartDashboard.putNumber("Target position", Robot.arm.posAbsZero); 
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
