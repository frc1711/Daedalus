/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  //MOTORS
  //drive base controllers
  public static int RRD = 4; 
  public static int RLD = 2; 
  public static int FLD = 1; 
  public static int FRD = 3; 

  //manipulator and arm talons 
  public static int manipulatorTalon = 6; 
  public static int armTalon = 5; 

  //lift
  public static int auxWheelTalon = 0; 

  //relays
  public static int hatchRelay = 1;

  //solenoids
  public static int armSolenoid = 1; 
  public static int armSolenoidRear = 5; 
  public static int botLift = 3; 
  public static int botLiftRear = 7; 
  public static int auxWheelSolenoid = 2;
  public static int auxWheelSolenoidRear = 6;  
  public static int botUnlock = 4; 
  public static int botUnlockRear = 0; 

  //controllers
  public static int controllerZero = 0;
  public static int controllerOne = 1; 

  //digital ports
  public static int manipulatorSwitch = 5; 

  //enums
  public enum RoboDir { 
    LEFT (-1), 
    RIGHT (1), 
    STRAIGHT(0);
    
    private int num; 

    private RoboDir(int num) {
      this.num = num; 
    }

    public int getNum() {
      return this.num; 
    }
  };

 
  

}
