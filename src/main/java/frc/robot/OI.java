/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new Example  Command());

>>>>>>> 545f5dca98d9702b3fc825682399047d239f3bc9
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
  public static JoystickButton modeToggleButton = new JoystickButton(controllerTwo
}
