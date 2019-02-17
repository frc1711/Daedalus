/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class ManipulatorHatch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Relay hatchRelay; 

  public ManipulatorHatch() {
    hatchRelay = new Relay(RobotMap.hatchRelay); 
  }

  public void setHatchRelay(Relay.Value state) { 
    hatchRelay.set(state); 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
