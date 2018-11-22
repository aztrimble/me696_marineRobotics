package org.ros.android.android_kanaloa_location;

import org.ros.android.*;
import org.ros.node.*;
import org.ros.namespace.*;
import org.ros.node.topic.*;
import org.ros.message.*;
import android.location.*;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.content.Context;
import android.os.Bundle;


public class AndroidPhoneGpsNode extends AbstractNodeMain {
  // Android stuff
  private Context mContext;

  
  // ROS stuff
  private Publisher publisher;
  private String publisherTopic;

  // raw GPS readings
  private double mLatitudeReading;
  private double mLongitudeReading;
  private double mAltitudeReading;
  private float mAccuracyReading;
  private long mTimeReading;
  private byte mGpsStatus;

  // ROS nsf_msgs Covariance
  private double[] positionCovariance = { 0, 0, 0,
                                          0, 0, 0,
                                          0, 0, 0};

  public AndroidPhoneGpsNode(Context mContext){
    // init android stuff
    this.mContext = mContext;

    // init ROS stuff
    publisherTopic="kanaloa/android/gps";
  }


  /*------------------ ROS stuff ------------------*/
  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("kanaloa/android_imu");
  }

  @Override
  public void onStart(final ConnectedNode connectedNode) {
    // init ROS publisher
    publisher = connectedNode.newPublisher(publisherTopic, sensor_msgs.NavSatFix._TYPE);
  }

  @Override
  public void onShutdown(Node node) {
    // shutdown ROS publisher
    publisher.shutdown();

  }
  
  @Override        
  public void onShutdownComplete(Node node) {
    publisher = null;
  }



  /*------------------ other helper stuff ------------------*/
  public void updateLocationInfo(Location loc) {
    if(loc == null) {
        mGpsStatus = sensor_msgs.NavSatStatus.STATUS_NO_FIX;
    } else {
        mLatitudeReading = loc.getLatitude();
        mLongitudeReading = loc.getLongitude();
        mAltitudeReading = loc.getAltitude();
        mAccuracyReading = loc.getAccuracy();
        mTimeReading = loc.getTime();
        mGpsStatus = sensor_msgs.NavSatStatus.STATUS_FIX;
    }
    sendGpsMsgsToRos();
  }

  public void sendGpsMsgsToRos(){
    if (publisher != null) {
      sensor_msgs.NavSatFix nsf_msg = (sensor_msgs.NavSatFix)publisher.newMessage();

      updatePositionCovariance();

      nsf_msg.setLatitude(mLatitudeReading);
      nsf_msg.setLongitude(mLongitudeReading);
      nsf_msg.setAltitude(mAltitudeReading);
      nsf_msg.setPositionCovariance(positionCovariance);
      nsf_msg.setPositionCovarianceType(sensor_msgs.NavSatFix.COVARIANCE_TYPE_APPROXIMATED);

      nsf_msg.getStatus().setStatus(sensor_msgs.NavSatStatus.STATUS_FIX);
      nsf_msg.getStatus().setService(sensor_msgs.NavSatStatus.SERVICE_GPS);

      nsf_msg.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
      nsf_msg.getHeader().setFrameId("android_frame");

      publisher.publish(nsf_msg);
    }
  }

  public void updatePositionCovariance(){
      // getAccuracy() returns a single float in meters,
      // so need to square it and update the diagonals
      double cov = mAccuracyReading * mAccuracyReading;

      positionCovariance[0] = cov;
      positionCovariance[4] = cov;
      positionCovariance[8] = cov;
  }

  public String gpsStatusToString(){
      if (mGpsStatus == sensor_msgs.NavSatStatus.STATUS_FIX)
          return "Fix";
      if (mGpsStatus == sensor_msgs.NavSatStatus.STATUS_NO_FIX)
          return "No Fix";

      return "Not Registered";
  }

  public String getMessage(){
      StringBuilder builder = new StringBuilder();

      builder.append("\n\n");
      builder.append("GPS: ");
      builder.append("\n");
      builder.append("     Latitude: ");
      builder.append(mLatitudeReading);
      builder.append("\n");
      builder.append("     Longitude: ");
      builder.append(mLongitudeReading);
      builder.append("\n");
      builder.append("     Altitude: ");
      builder.append(mAltitudeReading);
      builder.append("\n");
      builder.append("     Status: ");
      builder.append(gpsStatusToString());
      builder.append("\n\n");

    return builder.toString();
  }

}
