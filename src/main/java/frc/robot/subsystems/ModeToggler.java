/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ModeToggler extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Servo modeToggleDevice; 
  public ModeToggler() { 
    modeToggleDevice = new Servo(RobotMap.servoMode); 
  }

  public void runServo(double speed) {
    modeToggleDevice.set(speed); 
  }

  public void angleServo(double angle) {
    modeToggleDevice.setAngle(angle); 
  }

  public double getServoAngle() {
    return modeToggleDevice.getAngle(); 
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}