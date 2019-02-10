/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;

/**
 * @author: Spencer Crawford
 */
public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public DoubleSolenoid climbCylinder;
  public DoubleSolenoid pinCylinder;
  public DoubleSolenoid wheelCylinder;
  public DoubleSolenoid armCylinder;
  public Relay hatchCylinder;

  public Pneumatics(){

    //Double Action Solenoids are FORWARD, REVERSE, or OFF, and require 2 ports
    climbCylinder = new DoubleSolenoid(RobotMap.climbCylinderFront,RobotMap.climbCylinderRear);
    pinCylinder = new DoubleSolenoid(RobotMap.pinCylinderFront,RobotMap.pinCylinderRear);
    wheelCylinder = new DoubleSolenoid(RobotMap.wheelCylinderFront,RobotMap.wheelCylinderRear);
    armCylinder = new DoubleSolenoid(RobotMap.armCylinderFront,RobotMap.armCylinderRear);
    hatchCylinder = new Relay(RobotMap.hatchCylinder);
  }
  
  //Toggle Single Action Solenoids by setting position to opposite of current state

  //Set the state of Double Action Solenoids to "off", "forward", and "reverse".


  public void setArmCylinder(Value state){
    armCylinder.set(state);
  }

  public void setPinCylinder(Value state){
    pinCylinder.set(state);
  }

  public void setWheelCylinder(Value state){
    wheelCylinder.set(state);
  }

  public void setClimbCylinder(Value state){
    climbCylinder.set(state);
  }

  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
