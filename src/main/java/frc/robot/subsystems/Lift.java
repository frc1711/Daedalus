/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Lift extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public WPI_TalonSRX auxWheelTalon; 
  public DoubleSolenoid auxWheelSolenoid; 
  public DoubleSolenoid botLift; 
  public DoubleSolenoid unlockBot; 

  public Lift () {
    auxWheelTalon = new WPI_TalonSRX(RobotMap.auxWheelTalon); 
    botLift = new DoubleSolenoid(RobotMap.botLift, RobotMap.botLift); 
    unlockBot = new DoubleSolenoid(RobotMap.botUnlock, RobotMap.botUnlock); 
  }

  public void stopWheel() {
    auxWheelTalon.set(0); 
  }

  public void runAuxWheel(double speed) {
    auxWheelTalon.set(speed); 
  }
  
  public void setBotLift(Value state) {
    botLift.set(state); 
  }

  public void unlockBotLift(Value state) {
    unlockBot.set(state); 
  }

  public void setAuxWheel(Value state) { 
    auxWheelSolenoid.set(state); 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
