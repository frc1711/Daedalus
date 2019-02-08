/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Pneumatics extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Solenoid climbCylinder;
  public Solenoid pinCylinder;
  public Solenoid wheelCylinder;
  public DoubleSolenoid hatchCylinder;
  public DoubleSolenoid armCylinder;

  public Pneumatics(){

    //Single Action Solenoids are either ON or OFF, which requires one port
    climbCylinder = new Solenoid(RobotMap.climbCylinder);
    pinCylinder = new Solenoid(RobotMap.pinCylinder);
    wheelCylinder = new Solenoid(RobotMap.wheelCylinder);

    //Double Action Solenoids are FORWARD, REVERSE, or OFF, and require 2 ports
    hatchCylinder = new DoubleSolenoid(RobotMap.hatchCylinderF,RobotMap.hatchCylinderR);
    armCylinder = new DoubleSolenoid(RobotMap.armCylinderF,RobotMap.armCylinderR);
  }
  
  //Toggle Single Action Solenoids by setting position to opposite of current state
  public void toggleClimbCylinder(){
    climbCylinder.set(!climbCylinder.get());
  }

  public void togglePinCylinder(){
    pinCylinder.set(!pinCylinder.get());
  }

  public void toggleWheelCylinder(){
    wheelCylinder.set(!wheelCylinder.get());
  }

  //Set the state of Double Action Solenoids to "off", "forward", and "reverse".
  public void setHatchCylinder(Value state){
    hatchCylinder.set(state);
  }

  public void setArmCylinder(Value state){
    armCylinder.set(state);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
