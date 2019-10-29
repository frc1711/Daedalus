/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;


import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DoubleSolenoid;

import frc.robot.OI;
import frc.robot.Robot;

public class RunMotorArm extends Command {
  public boolean hold = false; 
  public boolean started = false; 
  public boolean state = false; 

  public RunMotorArm() {
    
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.arm); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   
   // Robot.clock.stopClock();     
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     
     SmartDashboard.putNumber("Enc position", Robot.arm.getSensorValue()); 
    
    //HATCH POSITIONS
    System.out.println(Robot.arm.getControllerMode()); 
    if(OI.armPosOne.get() && OI.controllerOne.getRawAxis(2) > .1) {
     //1
      Robot.arm.armTalon.selectProfileSlot(1, 0); 
      
      Robot.arm.runPIDArm(Robot.arm.hatchPosOne); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosOne); 
   
    } else if (OI.armPosTwo.get() && OI.controllerOne.getRawAxis(2) > .1 && Robot.arm.getControllerMode()) {
      //2
      Robot.arm.armTalon.selectProfileSlot(1, 0); 

      Robot.arm.runPIDArm(Robot.arm.hatchPosTwo); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosTwo); 

    } else if (OI.armPosThree.get() && OI.controllerOne.getRawAxis(2) > .1 && Robot.arm.getControllerMode()) {
     //3
      Robot.arm.armTalon.selectProfileSlot(1, 0); 

      Robot.arm.runPIDArm(Robot.arm.hatchPosThree); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosThree); 
    
    } else if (OI.armPosZero.get() && OI.controllerOne.getRawAxis(2) > .1) {
      Robot.arm.runPIDArm(Robot.arm.hatchPosOne); 
    }

    else if (OI.armPosZero.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      //hatch lift up
      Robot.arm.armTalon.selectProfileSlot(1, 0);

      Robot.arm.runPIDArm(Robot.arm.hatchLift);
  
      SmartDashboard.putNumber("Target position", Robot.arm.hatchLift); 

    } 
    //Cargo positions
    else if (OI.armPosOne.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      //1
      Robot.arm.armTalon.selectProfileSlot(1, 0);

      Robot.arm.runPIDArm(Robot.arm.cargoPosOne);

      SmartDashboard.putNumber("Target position", Robot.arm.cargoPosOne);  
    
    } else if (OI.armPosTwo.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      //2
      Robot.arm.armTalon.selectProfileSlot(1, 0);

      Robot.arm.runPIDArm(Robot.arm.cargoPosTwo);

      SmartDashboard.putNumber("Target position", Robot.arm.cargoPosTwo); 
    
    } else if (OI.armPosThree.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      //3
      Robot.arm.armTalon.selectProfileSlot(1, 0);

      Robot.arm.runPIDArm(Robot.arm.cargoPosThree); 

      SmartDashboard.putNumber("Target position", Robot.arm.cargoPosThree);

    } else if (OI.controllerOne.getPOV() == 90 && !(OI.controllerOne.getRawAxis(2) > .1)) {

      Robot.arm.runPIDArm(Robot.arm.home); 
     
      SmartDashboard.putNumber("Target position", Robot.arm.home); 
    
    } else if (OI.controllerOne.getPOV() == 90 && OI.controllerOne.getRawAxis(2) > .1) {
      Robot.arm.runPIDArm(Robot.arm.posZero); 
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