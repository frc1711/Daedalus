/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.subsystems.Arm;

public class RunMotorArm extends Command {
  private Arm arm; 
  private Joystick stick; 

  public RunMotorArm(Joystick stick) {
    requires(Robot.arm); 
    this.arm = Robot.arm; 
    this.stick = stick; 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     
    SmartDashboard.putNumber("Enc position", arm.getSensorValue()); 
    
    //HATCH POSITIONS
    if(stick.getRawButton(1) && stick.getRawAxis(2) > .1) {
     //1      
      arm.runPIDArm(RobotMap.hatchPosOne); 

      SmartDashboard.putNumber("Target position", RobotMap.hatchPosOne); 
   
    } else if (stick.getRawButton(3) && stick.getRawAxis(2) > .1 && arm.getControllerMode()) {
      //2
      arm.runPIDArm(RobotMap.hatchPosTwo); 

      SmartDashboard.putNumber("Target position", RobotMap.hatchPosTwo); 

    } else if (stick.getRawButton(4) && stick.getRawAxis(2) > .1 && arm.getControllerMode()) {
     //3
      arm.runPIDArm(RobotMap.hatchPosThree); 

      SmartDashboard.putNumber("Target position", RobotMap.hatchPosThree); 
    
    } else if (stick.getRawButton(2) && stick.getRawAxis(2) > .1) {
      arm.runPIDArm(RobotMap.hatchPosOne); 
    }

    else if (stick.getRawButton(2) && !(stick.getRawAxis(2) > .1)) {
      //hatch lift up
      arm.runPIDArm(RobotMap.hatchLift);
  
      SmartDashboard.putNumber("Target position", RobotMap.hatchLift); 

    } else if (stick.getPOV() == 90 && stick.getRawAxis(2) > .1) {
      //hatch home
      arm.runPIDArm(RobotMap.posZero); 
    }
    

    //Cargo positions
    else if (stick.getRawButton(1) && !(stick.getRawAxis(2) > .1)) {
      //1
      arm.runPIDArm(RobotMap.cargoPosOne);

      SmartDashboard.putNumber("Target position", RobotMap.cargoPosOne);  
    
    } else if (stick.getRawButton(3) && !(stick.getRawAxis(2) > .1)) {
      //2
      arm.runPIDArm(RobotMap.cargoPosTwo);

      SmartDashboard.putNumber("Target position", RobotMap.cargoPosTwo); 
    
    } else if (stick.getRawButton(4) && !(stick.getRawAxis(2) > .1)) {
      //3
      arm.runPIDArm(RobotMap.cargoPosThree); 

      SmartDashboard.putNumber("Target position", RobotMap.cargoPosThree);

    } else if (stick.getPOV() == 90 && !(stick.getRawAxis(2) > .1)) {

      arm.runPIDArm(RobotMap.home); 
     
      SmartDashboard.putNumber("Target position", RobotMap.home); 
    
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