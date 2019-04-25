/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new Example  Command());

  public static Joystick controllerZero = new Joystick(RobotMap.controllerZero);
  public static Joystick controllerOne = new Joystick(RobotMap.controllerOne); 
  public static Joystick controllerTwo = new Joystick(RobotMap.controllerTwo); 

  //BUTTONS
  //controller zero, drive 
  public static JoystickButton lineUpEnable = new JoystickButton(controllerZero, 4); 
  public static JoystickButton ballVisionEnable = new JoystickButton(controllerZero, 2);
  public static JoystickButton speedButton = new JoystickButton(controllerZero, 1); 
  public static JoystickButton manipButtonZero = new JoystickButton(controllerZero, 5); 
  public static JoystickButton manipButtonOne = new JoystickButton(controllerZero, 6);
  public static JoystickButton climbSafetyZero = new JoystickButton(controllerZero, 7); 
  public static JoystickButton climbSafetyOne = new JoystickButton(controllerZero, 8); 
  
  //controller one, manipulator
  public static JoystickButton armPosZero = new JoystickButton(controllerOne, 2); 
  public static JoystickButton armPosOne = new JoystickButton(controllerOne, 1);
  public static JoystickButton armPosTwo = new JoystickButton(controllerOne, 3); 
  public static JoystickButton armPosThree = new JoystickButton(controllerOne, 4); 
  public static JoystickButton liftDeployZero = new JoystickButton(controllerOne, 5); 
  public static JoystickButton liftDeployOne = new JoystickButton(controllerOne, 6); 
  public static JoystickButton pnuematicOff = new JoystickButton(controllerOne, 7); 
  public static JoystickButton pnuematicOffTwo = new JoystickButton(controllerOne, 8); 
  public static JoystickButton auxWheelButton = new JoystickButton(controllerOne, 10); 
 
  //controller two, toggle
  public static JoystickButton modeToggleButton = new JoystickButton(controllerTwo, 2); 
}
