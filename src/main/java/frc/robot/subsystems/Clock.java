/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Clock extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Timer clockTimer; 

  public Clock() {
    clockTimer = new Timer(); 
  }
  
  public void resetClock() {
    clockTimer.reset(); 
  }

  public void startClock() {
    clockTimer.start(); 
  }

  public void stopClock() {
    clockTimer.stop(); 
  }

  public double getClock() {
    return clockTimer.get();
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
