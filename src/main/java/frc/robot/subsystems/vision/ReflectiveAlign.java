/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems.vision;


import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
/**
 * Add your docs here.
 */

public class ReflectiveAlign {
   /* private static ArrayList<Block> blocks = CameraConfig.getPixyCamera().getPixy().getCCC().getBlocks();
    private static final int blockSignature = 1; 
    public static Block largestBlock = null; 
    public static Block smallestBlock = null; 
    public static double yaw; 

    public static void run(int count) {
       /* if (count > 0 ) {
            Block largestBlock = null; 
            Block smallestBlock = null; 
            for (Block block : blocks) {
                if(block.getSignature() == blockSignature) {
                    //i think this needs to be changed but who knows
                    if (largestBlock == null && smallestBlock == null) {
                        largestBlock = block; 
                        smallestBlock = block; 
                    }
                    else if (block.getWidth() > largestBlock.getWidth()) 
                        largestBlock = block;
                    else if (block.getWidth() < largestBlock.getWidth())
                        smallestBlock = block; 
                } 
            }

            
            int tapeXLarge = largestBlock.getX(); 
            int tapeXSmall = smallestBlock.getX(); 
            int averageX = (tapeXLarge + tapeXSmall) / 2; 
            double yaw = ((averageX - 157.5) * 0.1904761905);

            SmartDashboard.putNumber("Large X, Tape", largestBlock.getX()); 
            SmartDashboard.putNumber("Small X, Tape", smallestBlock.getX()); 
            SmartDashboard.putNumber("Average X, Tape", averageX);
            SmartDashboard.putNumber("Tape Angle?", yaw); 
            SmartDashboard.putNumber("Large Tape Width", largestBlock.getWidth()); 
            SmartDashboard.putNumber("Small Tape Width", smallestBlock.getWidth());
            SmartDashboard.putNumber("Large Tape Height", largestBlock.getHeight());
            SmartDashboard.putNumber("Smalal Tape Height", smallestBlock.getHeight()); 
        }
    } 
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
  
        SmartDashboard.putNumber("Ball Angle", yaw);
        SmartDashboard.putNumber("Ball X", largestBlock.getX());
        SmartDashboard.putNumber("Ball Y", largestBlock.getY());
        SmartDashboard.putNumber("Ball Box Width", largestBlock.getWidth());
        SmartDashboard.putNumber("Ball Box Height", largestBlock.getHeight());
      }
    } */
}