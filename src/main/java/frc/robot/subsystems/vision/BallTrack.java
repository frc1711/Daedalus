/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Dashboard;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;

/**
 * Add your docs here.
 */
public class BallTrack {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static ArrayList<Block> blocks = CameraConfig.getPixyCamera().getPixy().getCCC().getBlocks();
  private static final int blockSignature = 1; 

  public static void run(int count) {
    if (count > 0) {
      Block largestBlock = null; 
      
      for (Block block : blocks) {
        if(block.getSignature() == blockSignature) {
          if (largestBlock == null) 
            largestBlock = block; 
          else if (block.getWidth() > largestBlock.getWidth()) 
            largestBlock = block; 
        }
      }

      int ballX = largestBlock.getX(); 
      double yaw = ((ballX - 157.5) * 0.1904761905); 
    
      Dashboard.getInstance().putNumber(false, "Ball Angle", yaw);
      Dashboard.getInstance().putNumber(false, "Ball X", largestBlock.getX());
      Dashboard.getInstance().putNumber(false, "Ball Y", largestBlock.getY());
      Dashboard.getInstance().putNumber(false, "Ball Box Width", largestBlock.getWidth());
      Dashboard.getInstance().putNumber(false, "Ball Box Height", largestBlock.getHeight());
    }
  }
}
