// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.LimeLightConstants;
// lime light subsystem



public class GridLimelightSubsystem {

   private static NetworkTableInstance m_instance = NetworkTableInstance.getDefault();
   private static NetworkTable m_table = m_instance.getTable("limelight-grid");
   private static DoubleSubscriber m_IN_tx = m_table.getDoubleTopic("tx").subscribe(0.0);  // Horizontal offset from crosshair to target (-29.8 to 29.8 degrees).
   private static DoubleSubscriber m_IN_ty = m_table.getDoubleTopic("ty").subscribe(0.0);  // Vertical offset from crosshair to target (-24.85 to 24.85 degrees).
   private static DoubleSubscriber m_IN_tv = m_table.getDoubleTopic("tv").subscribe(0.0);  // Whether the limelight has any valid targets (0 or 1).
   private static DoubleSubscriber m_IN_ta = m_table.getDoubleTopic("ta").subscribe(0.0);  // Target area (0% of image to 100% of image).
   private static DoubleSubscriber m_IN_tl = m_table.getDoubleTopic("tl").subscribe(0.0);  //The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
   //private static DoubleSubscriber m_IN_ledMode = m_table.getDoubleTopic("ledMode").subscribe(0.0); // limelight's LED state (0-3).
   private static DoubleSubscriber m_IN_pipeline = m_table.getDoubleTopic("pipeline").subscribe(0.0); // returns the pipeline the limelight has selected
   private static int m_Robot_Mode;
   private static DoublePublisher m_OUT_ledMode;
   // private static DoublePublisher m_OUT_pipeline;
   // m_Robot_Mode = Constants.RobotMode.kCone; // default is cone until changed 
      
  /**
   * Gets target latency (ms).
   * 
   * return Target latency.
   */
  public static double getTl() 
  {
   return m_IN_tl.getAsDouble();
  }

 /**
  * Get the horizontal offset from the Limelight
  *
  * @return tx from the Limelight NetworkTable
  */
 public static double getTx() {
    return m_IN_tx.getAsDouble();
 }

 /**
  * Get the vertical offset from the Limelight
  *
  * @return ty from the Limelight NetworkTable
  */
 public static double getTy() 
 {
   return m_IN_ty.getAsDouble();
 }

 /**
  * Get the target area from the limelight
  *
  * @return ta from the Limelight NetworkTable
  */
 public static double getTa()
 {
   return m_IN_ta.getAsDouble();
 }

 /**
  * Get the target verification from the Limelight
  *
  * @return tv from the Limelight NetworkTable
  */
 public static double getTv() 
 {
   return m_IN_tv.getAsDouble();
 }

 /**
  * Check to see if Limelight has a target
  * 
  * @return True if Limelight has a target
  */
 public static boolean hasTarget()
 {
    //log();
    return getTv() == 1.0;
 }

 /**
  * Return the robot LL mode
  * 
  * @return Return the robot mode 0 for cone, 1 for cube
  */
 public static double getRobotMode() 
 {
   return m_Robot_Mode;
 }

 /**
  * sets the  robot Limelight  mode cones or cube  0 for cone, 1 for cube
  * 
  * @return sets the  robot Limelight  mode cones or cube  0 for cone, 1 for cube
  */
 public static void setRobotMode(int setvalue) 
 {
   m_Robot_Mode = setvalue;
   
 }
 /** 
  * Gets the pipeline the LL is using
  * 
  * @return the pipeline 3 means it failed, pushes to the SmartDashboard
  */
 public static double getLLPipeline() 
 {
   SmartDashboard.putNumber("pipelineNumber", m_IN_pipeline.getAsDouble());
   return m_IN_pipeline.getAsDouble();
 }

 /**
  * Set the pipeline the LL is using
  * 
  * Sets the network table pipeline to use, pushes the number to smartdashboard
  */
 public static void setLLPipeline(double setvalue) 
 {
    //m_OUT_pipeline.set(setvalue);   
    m_table.getEntry("pipeline").setNumber(setvalue);
    SmartDashboard.putNumber("pipelineNumber", m_IN_pipeline.getAsDouble()); 
 }

 /**
  * Methods for external classes to change green light's status.
  */
 public static void turnOnLED()
 {
  m_OUT_ledMode.set(3);    //Sets limelight’s LED state,  0	use the LED Mode set in the current pipeline,  1	force off, 2	force blink, 3	force on
 }                 
 public static void turnOffLED()
 {
  m_OUT_ledMode.set(1);
 }
 public static void blinkLED()
 {
  m_OUT_ledMode.set(2);
 }

 public static double getDistanceMeters() {
  //double offset = 0;
  if (hasTarget()) {
    double h2;
    if(getTy() < 0) {
      h2 = Constants.LimeLightConstants.kLowTargetHeightH2;
    } else {
      h2 = Constants.LimeLightConstants.kHighTargetHeightH2;
    }
    double h1 = Constants.LimeLightConstants.kCameraHeightH1;
    double a1 = Math.toRadians(Constants.LimeLightConstants.kCameraAngle);
    double a2 = Math.toRadians(getTy());

    return (double) ((h2 - h1) / (Math.tan(a1 + a2)));
  }
  return 0.0;
}

public static double getXDistanceMetersGridEdge() {
  double offset;
  if (hasTarget()) {
    if(getTy() < 0) {
      offset = Constants.LimeLightConstants.kLowTargetGridDistance;
    } else {
      offset = Constants.LimeLightConstants.kHighTargetGridDistance;
    }
    return getDistanceMeters() - (offset + Constants.LimeLightConstants.kGridCameraDepth);
  }
  return 0.0;
}

public static double leftRightDistanceMeters() {
  return getDistanceMeters() * Math.tan(getTx() / 360 * Math.PI * 2) - LimeLightConstants.kCameraOffsetDistance;
}

public static double getTxOffsetRadians() {
  return Math.atan(Constants.LimeLightConstants.kCameraOffsetDistance / getDistanceMeters());
}

public static double getTxOffsetDegrees() {
  return getTxOffsetRadians() * 360 / (2 * Math.PI);
}

 /**
  * Send current limelight NetworkTable entries to the SmartDashboard
  */
 public void log()
 {
   SmartDashboard.putNumber("GridLimelight X", getTx());
   SmartDashboard.putNumber("GridLimelight Y", getTy());
   SmartDashboard.putNumber("GridLimelight Area", getTa());
  }
}
