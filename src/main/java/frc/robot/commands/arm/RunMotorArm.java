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
     //double sensorVel = Robot.arm.armTalon.getSelectedSensorVelocity(); 
     //double MOP = Robot.arm.armTalon.getMotorOutputPercent(); 
    // SmartDashboard.putNumber("Sensor Velocity", sensorVel); 
     //SmartDashboard.putNumber("armTalon Motor Output Percent", MOP); 
     SmartDashboard.putNumber("Enc position", Robot.arm.getSensorValue()); 
     //double speed = OI.controllerOne.getRawAxis(1) * OI.controllerOne.getRawAxis(1); 
    // double runningSpeed = speed / 2; 
     //SmartDashboard.putNumber("Arm speed", runningSpeed); 
     //double iGAIN = Robot.arm.armTalon.getIntegralAccumulator(); 
     //double cle = Robot.arm.armTalon.getClosedLoopError(); 
    //SmartDashboard.putNumber("CLOSED LOOP ERROR", cle); 
    //SmartDashboard.putNumber("INTGAIN", iGAIN); 
  //  if () {
   // Robot.modeToggler.modeToggleDevice.setAngle(180); 
      //Robot.modeToggler.runServo(OI.controllerTwo.getRawAxis(1) ); 
    // if (OI.modeToggleButton.get() && !state) {

   //  } else if (OI.modeToggleButton.get() && state) {
       //Robot.modeToggler.angleServo(0); 
    // }


//}
    //HATCH POSITIONS
    System.out.println(Robot.arm.getControllerMode()); 
    if(OI.armPosOne.get() && OI.controllerOne.getRawAxis(2) > .1) {
     //1
      Robot.arm.armTalon.selectProfileSlot(1, 0); 
      
     // Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
      Robot.arm.runPIDArm(Robot.arm.hatchPosOne); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosOne); 
   
    } else if (OI.armPosTwo.get() && OI.controllerOne.getRawAxis(2) > .1 && Robot.arm.getControllerMode()) {
      //2
      Robot.arm.armTalon.selectProfileSlot(1, 0); 
      //Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kForward); 
      Robot.arm.runPIDArm(Robot.arm.hatchPosTwo); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosTwo); 

    } else if (OI.armPosThree.get() && OI.controllerOne.getRawAxis(2) > .1 && Robot.arm.getControllerMode()) {
     //3
      Robot.arm.armTalon.selectProfileSlot(1, 0); 
     // Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
      Robot.arm.runPIDArm(Robot.arm.hatchPosThree); 

      SmartDashboard.putNumber("Target position", Robot.arm.hatchPosThree); 
    
    } else if (OI.armPosZero.get() && OI.controllerOne.getRawAxis(2) > .1) {
      Robot.arm.runPIDArm(Robot.arm.hatchPosOne); 
    }
      //CARGO POSITIONS
    else if (OI.armPosZero.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      //1
      Robot.arm.armTalon.selectProfileSlot(1, 0);
     // Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
      Robot.arm.runPIDArm(Robot.arm.hatchLift);
      SmartDashboard.putNumber("Target position", Robot.arm.hatchLift); 

    } else if (OI.armPosOne.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      //1
      Robot.arm.armTalon.selectProfileSlot(1, 0);
     // Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kReverse); 
      Robot.arm.runPIDArm(Robot.arm.posOne);
      SmartDashboard.putNumber("Target position", Robot.arm.posOne);  
    
    } else if (OI.armPosTwo.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      //2
      Robot.arm.armTalon.selectProfileSlot(1, 0);
     // Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kForward); 
      Robot.arm.runPIDArm(Robot.arm.posTwo);
      SmartDashboard.putNumber("Target position", Robot.arm.posTwo); 
    
    } else if (OI.armPosThree.get() && !(OI.controllerOne.getRawAxis(2) > .1)) {
      //3
      Robot.arm.armTalon.selectProfileSlot(1, 0);
      //Robot.pneumaticArm.armSolenoid.set(DoubleSolenoid.Value.kForward); 
      Robot.arm.runPIDArm(Robot.arm.posThree); 
      SmartDashboard.putNumber("Target position", Robot.arm.posThree);

    } else if (OI.controllerOne.getPOV() == 90 && !(OI.controllerOne.getRawAxis(2) > .1)) {

      Robot.arm.runPIDArm(Robot.arm.posAbsZero); 
      SmartDashboard.putNumber("Target position", Robot.arm.posAbsZero); 
    
    } else if (OI.controllerOne.getPOV() == 90 && OI.controllerOne.getRawAxis(2) > .1) {
      Robot.arm.runPIDArm(Robot.arm.posZero); 
    }

    //double armTargetPos = SmartDashboard.getNumber("Target position", 0); 
  // double encCount = SmartDashboard.getNumber("Enc position", 10); 
    //if (!(armTargetPos == 0))
      //Robot.arm.stopPIDPos(sensorVel, encCount, armTargetPos, MOP);  
   // else if (armTargetPos == 0) {
   //   if (encCount >= armTargetPos - 50 && encCount <= armTargetPos + 50 && sensorVel == 0) {    }
      //  Robot.arm.runArm(-.015); 
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