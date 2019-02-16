/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Manipulator extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  TalonSRX manipulatorTalon; 
  Relay hatchRelay; 
  DigitalInput manipulatorSwitch; 

  public Manipulator() {
    manipulatorTalon = new TalonSRX(RobotMap.manipulatorTalon);
    hatchRelay = new Relay(RobotMap.hatchRelay); 
    manipulatorSwitch = new DigitalInput(RobotMap.manipulatorSwitch);
  }

  public void runManipulator(double speed) {
    manipulatorTalon.set(ControlMode.PercentOutput, speed);
  }
  
  public void setHatchRelay(Relay.Value state) {
    hatchRelay.set(state); 
  }
  
  public boolean getManipulatorSwitch() {
    return manipulatorSwitch.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
