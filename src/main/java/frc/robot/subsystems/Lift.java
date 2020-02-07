/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.commands.lift.ScissorLift;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  private DoubleSolenoid botLift; 
  private DoubleSolenoid unlockBot; 
  private Joystick stick; 

  public Lift (Joystick stick) {
    botLift = new DoubleSolenoid(RobotMap.botLift, RobotMap.botLiftRear); 
    unlockBot = new DoubleSolenoid(RobotMap.botUnlock, RobotMap.botUnlockRear); 
    this.stick = stick; 
  }
  
  public void setBotLift(DoubleSolenoid.Value state) {
    botLift.set(state); 
  }

  public void unlockBotLift(DoubleSolenoid.Value state) {
    unlockBot.set(state); 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new ScissorLift(stick));
  }
}
