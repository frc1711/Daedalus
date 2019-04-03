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
import frc.robot.subsystems.PID.AntiWindArm;

public class RunPIDArm extends Command {
  public RunPIDArm() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.antiWindArm); 
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    SmartDashboard.putNumber("Enc Pos", Robot.antiWindArm.armTalon.getSelectedSensorPosition()); 
    Robot.antiWindArm.positionControl();
    double speed = Robot.antiWindArm.motorDC; 
    //Robot.antiWindArm.runArmTalon(speed);
    SmartDashboard.putNumber("PID Speed", speed); 
    SmartDashboard.putNumber("PID Targ", Robot.antiWindArm.getTargetPosition()); 
    SmartDashboard.putNumber("PID Location", Robot.antiWindArm.getCurrentPosition()); 
    if (OI.armPosZero.get() && Robot.antiWindArm.getTargetPosition() != 300) {
      Robot.antiWindArm.setTargetPosition(300); 
     // Robot.antiWindArm.runArmTalon(0.04); 
    } else if (OI.armPosOne.get() && Robot.antiWindArm.getTargetPosition() != 2026) {
      Robot.antiWindArm.setTargetPosition(2026); 
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