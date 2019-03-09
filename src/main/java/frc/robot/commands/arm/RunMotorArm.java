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
   double sensorVel = Robot.arm.armTalon.getSelectedSensorVelocity(); 
   double MOP = Robot.arm.armTalon.getMotorOutputPercent(); 
   SmartDashboard.putNumber("Sensor Velocity", sensorVel); 
   System.out.println("SENSOR VEL WORKING"); 
   SmartDashboard.putNumber("armTalon Motor Output Percent", MOP); 
   SmartDashboard.putNumber("Enc position", Robot.arm.getSensorValue()); 
   double speed = OI.controllerOne.getRawAxis(1) * OI.controllerOne.getRawAxis(1); 
   double runningSpeed = speed / 2; 
   SmartDashboard.putNumber("Arm speed", runningSpeed); 
   //System.out.println(Robot.arm.getSensorValue());

   //EXIT PID LOOP ON BUTTON RELEASE
    /*if (!OI.armPosZero.get() && !OI.armPosOne.get() && !OI.armPosTwo.get() && !OI.armPosThree.get() && !(OI.controllerOne.getPOV() == 90)) {
      double holdPos = SmartDashboard.getNumber("armTalon Motor Output Percent", 0); 
      Robot.arm.runArm(holdPos); 
    }  */

    //RUN THE ARM AT 1/3RD THE SPEED
   /*if (Math.abs(OI.controllerOne.getRawAxis(1)) > .1) {
      double armSpeedRun = OI.controllerOne.getRawAxis(1) / 3; 
      Robot.arm.runArm(armSpeedRun);
       
      SmartDashboard.putNumber("Arm Value", armSpeedRun); 
    }  */
   
    //HATCH POSITIONS
    if(OI.armPosOne.get() && OI.controllerOne.getRawAxis(2) > .1) {
     
      Robot.arm.armTalon.selectProfileSlot(1, 0); 
      Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
      Robot.arm.runPIDArm(Robot.arm.hatchPosOne); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosOne); 
   
    } else if (OI.armPosTwo.get() && OI.controllerOne.getRawAxis(2) > .1) {
      
      Robot.arm.armTalon.selectProfileSlot(2, 0); 
      Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kForward); 
      Robot.arm.runPIDArm(Robot.arm.hatchPosTwo); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosTwo); 

    } else if (OI.armPosThree.get() && OI.controllerOne.getRawAxis(2) > .1) {
     
      Robot.arm.armTalon.selectProfileSlot(3, 0); 
      Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
      Robot.arm.runPIDArm(Robot.arm.hatchPosThree); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosThree); 
    
    } //CARGO POSITIONS
    else if (OI.armPosZero.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {

      Robot.arm.armTalon.selectProfileSlot(1, 0);
      Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
      Robot.arm.runPIDArm(Robot.arm.posZero);
      SmartDashboard.putNumber("Target position", Robot.arm.posZero); 

    } else if (OI.armPosOne.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {

      Robot.arm.armTalon.selectProfileSlot(1, 0);
      Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
      Robot.arm.runPIDArm(Robot.arm.posOne);
      SmartDashboard.putNumber("Target position", Robot.arm.posOne);  
    
    } else if (OI.armPosTwo.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      
      Robot.arm.armTalon.selectProfileSlot(2, 0);
      Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kForward); 
      Robot.arm.runPIDArm(Robot.arm.posTwo);
      SmartDashboard.putNumber("Target position", Robot.arm.posTwo); 
    
    } else if (OI.armPosThree.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {

      Robot.arm.armTalon.selectProfileSlot(3, 0);
      Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kForward); 
      Robot.arm.runPIDArm(Robot.arm.posThree); 
      SmartDashboard.putNumber("Target position", Robot.arm.posThree);

    } else if (OI.controllerOne.getPOV() == 90) {

      Robot.arm.runPIDArm(Robot.arm.posAbsZero); 
      SmartDashboard.putNumber("Target position", Robot.arm.posAbsZero); 
    
    }

    /*if (vel == 0 ) {
      double holdPos = SmartDashboard.getNumber("armTalon Motor Output Percent", 0); 
      Robot.arm.runArm(holdPos); 
    }*/

    double armTargetPos = SmartDashboard.getNumber("Target position", 0); 
    double encCount = SmartDashboard.getNumber("Enc position", 10); 
    //if (!OI.armPosZero.get() && !OI.armPosTwo.get() && !OI.armPosThree.get() && !(OI.controllerOne.getPOV() == 180)) {
    Robot.arm.stopPIDPos(sensorVel, encCount, armTargetPos, MOP);  
    //}

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
