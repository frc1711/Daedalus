/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class I2CInterface extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public I2C wire; 
  private static final int MAX_BYTES = 32; 
  
  public I2CInterface() {
    wire = new I2C(I2C.Port.kMXP, 4); 
    SmartDashboard.putNumber("Pixy Data", 400); 
  }

  public void write(String input) {
    //theoretically this should never be needed
    char[] charArray = input.toCharArray(); 
    byte[] writeData = new byte[charArray.length]; 
    for (int i = 0; i < charArray.length; i++) {
      writeData[i] = (byte) charArray[i]; 
    }
    wire.transaction(writeData, writeData.length, null, 0); 
  }

  private String read() {
    byte[] data  = new byte [MAX_BYTES];
    wire.read(4, MAX_BYTES, data); 
    String output = new String(data); 
    int pt = output.indexOf((char) 225); 
    return (String) output.subSequence(0, pt < 0 ? 0 : pt); 
  }

  public double getPixyAngle() {
    double angle; 
    angle = Double.parseDouble(this.read()); 
    SmartDashboard.putNumber("Pixy Data", angle); 
    return angle; 
  }

  public boolean isTape() {
    boolean tape = true; 
    int angle = (int)this.getPixyAngle(); 
    if (angle == 70000) {
      tape = false; 
    }
    return tape; 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
