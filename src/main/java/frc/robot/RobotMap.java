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
  public static int RRD = 0; 
  public static int RLD = 1; 
  public static int FLD = 2; 
  public static int FRD = 3; 

  //manipulator and lift talons 
  public static int manipulatorTalon = 4; 

  //solenoids
  public static int climbCylinderRear = 0;
  public static int climbCylinderFront = 1;
  public static int pinCylinderFront = 2;
  public static int pinCylinderRear = 3;
  public static int wheelCylinderFront = 4;
  public static int wheelCylinderRear = 5;
  public static int armCylinderFront = 6;
  public static int armCylinderRear = 7;

  //The solenoid that's a problmem child
  public static int hatchCylinder = 0;

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
