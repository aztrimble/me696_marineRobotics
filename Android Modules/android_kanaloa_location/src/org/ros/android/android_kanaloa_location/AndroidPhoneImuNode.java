package org.ros.android.android_kanaloa_location;

import org.ros.android.*;
import org.ros.node.*;
import org.ros.namespace.*;
import org.ros.node.topic.*;
import org.ros.message.*;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.content.Context;


public class AndroidPhoneImuNode extends AbstractNodeMain implements SensorEventListener {
  // Constants
  //static final float ALPHA = 0.10f; //0.10f == Accelerometer & Magnetic Field
  static final float ALPHA = 0.32f; //0.32f == Rotation Vector

  // Android stuff
  private Context mContext;
  private SensorManager mSensorManager;
  
  // ROS stuff
  private Publisher publisherWorld;
  private String publisherTopicWorld;
  private Publisher publisherDevice;
  private String publisherTopicDevice;
  private Publisher publisherWorldVector3Stamped;
  private String publisherTopicWorldVector3Stamped;

  // accuaracy level of sensor
  private String accelerometerAccuracyLevel;
  private String magneticFieldAccuracyLevel;
  private String linearAccelerationAccuracyLevel;
  private String gyroscopeAccuracyLevel;
  private String rotationVectorAccuracyLevel;

  // raw data sensor reading
  private final float[] mAccelerometerReading = new float[3];
  private final float[] mMagnetometerReading = new float[3];
  private final float[] mLinearAccelerationReading = new float[3];
  private final float[] mGyroscopeReading = new float[3];
  private final float[] mRotationVectorReading = new float[3];
  private final float[] mGameRotationVectorReading = new float[4];

  // calculated
  private final float[] mRotationMatrix = new float[9];
  private final float[] mInclinationMatrix = new float[9];
  private final float[] mOrientationAngles = new float[3];
  private final float[] mOrientationQuaternion = new float[4];
  private final float[] mOrientationQuaternion_device = new float[4];
  private Float inclination;

  // ROS imumsgs Covariances
  private double[] orientationCovariance = { 0.002741552146694444, 0, 0,
                                             0, 0.002741552146694444, 0,
                                             0, 0, 0.007615422629706791 };
  private double[] angularCovariance = { 1.0966208586777776e-06, 0, 0,
                                         0, 1.0966208586777776e-06, 0,
                                         0, 0, 1.0966208586777776e-06 };
  private double[] linearCovariance = { 0.0015387262937311438, 0, 0,
                                        0, 0.0015387262937311438, 0,
                                        0, 0, 0.0015387262937311438 };

  public AndroidPhoneImuNode(Context mContext){
    // init android stuff
    this.mContext = mContext;
    mSensorManager = (SensorManager) mContext.getSystemService(Context.SENSOR_SERVICE);

    // init ROS stuff
    publisherTopicWorld="kanaloa/android/imu_world";
    publisherTopicDevice="kanaloa/android/imu_device";
    publisherTopicWorldVector3Stamped="kanaloa/android/rpy_world";

    // init reading accuracy level
    accelerometerAccuracyLevel = "notRegistered";
    magneticFieldAccuracyLevel = "notRegistered";
    linearAccelerationAccuracyLevel = "notRegistered";
    gyroscopeAccuracyLevel = "notRegistered";
    rotationVectorAccuracyLevel = "notRegistered";
  }

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy) {
    // TYPE_ROTATION_VECTOR sesnor, only avaliable if have a gyro scope on phone
    if(sensor == mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)) {
        rotationVectorAccuracyLevel = accuracyLevelToString(accuracy);
    }
    if(sensor == mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)) {
        rotationVectorAccuracyLevel = accuracyLevelToString(accuracy);
    }
    if(sensor == mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER)) {
        accelerometerAccuracyLevel = accuracyLevelToString(accuracy);
    }
    if(sensor == mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)) {
        magneticFieldAccuracyLevel = accuracyLevelToString(accuracy);
    }
    if(sensor == mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)) {
        linearAccelerationAccuracyLevel = accuracyLevelToString(accuracy);
    }
    if(sensor == mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)) {
        gyroscopeAccuracyLevel = accuracyLevelToString(accuracy);
    }
  }

  public String accuracyLevelToString(int currentAccuracyLevel){
     if (currentAccuracyLevel == mSensorManager.SENSOR_STATUS_ACCURACY_HIGH){
         return "High";
     } else if (currentAccuracyLevel == mSensorManager.SENSOR_STATUS_ACCURACY_MEDIUM) {
         return "Medium";
     } else if (currentAccuracyLevel == mSensorManager.SENSOR_STATUS_ACCURACY_LOW) {
         return "Low";
     } else if (currentAccuracyLevel == mSensorManager.SENSOR_STATUS_UNRELIABLE) {
         return "Unreliable";
     } else if (currentAccuracyLevel == mSensorManager.SENSOR_STATUS_NO_CONTACT) {
         return "No Contact";
     }
     return "unreliable";
  }

  @Override
  public GraphName getDefaultNodeName() {
      return GraphName.of("kanaloa/android_gps");
  }
  
  @Override
  public void onStart(final ConnectedNode connectedNode) {
  //public void onStart(final Node node) {
    // register publisher node to ROS
    //publisher = connectedNode.newPublisher(publisherTopic, std_msgs.String._TYPE);
    publisherWorld = connectedNode.newPublisher(publisherTopicWorld, sensor_msgs.Imu._TYPE);
    publisherDevice = connectedNode.newPublisher(publisherTopicDevice, sensor_msgs.Imu._TYPE);
    publisherWorldVector3Stamped = connectedNode.newPublisher(publisherTopicWorldVector3Stamped, geometry_msgs.Vector3Stamped._TYPE);

    // Get updates from the accelerometer and magnetometer at a constant rate.
    // To make batch operations more efficient and reduce power consumption,
    // provide support for delaying updates to the application.
    //
    // In this example, the sensor reporting delay is small enough such that
    // the application receives an update before the system checks the sensor
    // readings again.
    Sensor rotationVector = mSensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR);
    if (rotationVector != null) {
        mSensorManager.registerListener(this, rotationVector,
            SensorManager.SENSOR_DELAY_GAME);
    }
    Sensor gameRotationVector = mSensorManager.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR);
    if (gameRotationVector != null) {
        mSensorManager.registerListener(this, gameRotationVector,
            SensorManager.SENSOR_DELAY_GAME);
    }
    Sensor accelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
    if (rotationVector == null && accelerometer != null) {
        mSensorManager.registerListener(this, accelerometer,
            SensorManager.SENSOR_DELAY_GAME);
    }
    Sensor magneticField = mSensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);
    if (rotationVector == null && magneticField != null) {
        mSensorManager.registerListener(this, magneticField,
            SensorManager.SENSOR_DELAY_GAME);
    }
    Sensor linearAcceleration = mSensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
    if (linearAcceleration != null) {
        mSensorManager.registerListener(this, linearAcceleration,
            SensorManager.SENSOR_DELAY_GAME);
    }
    Sensor gyroscope = mSensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
    if (gyroscope != null) {
        mSensorManager.registerListener(this, gyroscope,
            SensorManager.SENSOR_DELAY_GAME);
    }
  }

  // Get readings from accelerometer and magnetometer. To simplify calculations,
  // consider storing these readings as unit vectors.
  @Override
  public void onSensorChanged(SensorEvent event) {
      if (event.sensor.getType() == Sensor.TYPE_ROTATION_VECTOR) {
        lowPass(event.values.clone(), mRotationVectorReading);
        updateOrientationAnglesFromVector();
      } 
      if (event.sensor.getType() == Sensor.TYPE_GAME_ROTATION_VECTOR) {
        System.arraycopy(event.values, 0, mGameRotationVectorReading, 0, mGameRotationVectorReading.length);
        //lowPass(event.values.clone(), mGameRotationVectorReading);
        updateOrientationAnglesFromVector();
      } 
      if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
        //System.arraycopy(event.values, 0, mAccelerometerReading, 0, mAccelerometerReading.length);
        lowPass(event.values.clone(), mAccelerometerReading);
        updateOrientationAngles();
      } 
      if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
        //System.arraycopy(event.values, 0, mMagnetometerReading, 0, mMagnetometerReading.length);
        lowPass(event.values.clone(), mMagnetometerReading);
        updateOrientationAngles();
      } 
      if (event.sensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {
        //System.arraycopy(event.values, 0, mLinearAccelerationReading, 0, mLinearAccelerationReading.length);
        lowPass(event.values.clone(), mLinearAccelerationReading);
      } 
      if (event.sensor.getType() == Sensor.TYPE_GYROSCOPE) {
        //System.arraycopy(event.values, 0, mGyroscopeReading, 0, mGyroscopeReading.length);
        lowPass(event.values.clone(), mGyroscopeReading);
      }
      
      // send info to ROS master
      sendWorldImuMsgsToRos(); 
      sendDeviceImuMsgsToRos(); 
      sendWorldVector3StampedMsgsToRos(); 
  }

  // Compute the three orientation angles based on the most recent readings from
  // the device's accelerometer and magnetometer.
  public void updateOrientationAngles() {
      // Update rotation matrix, which is needed to update orientation angles.
      SensorManager.getRotationMatrix(mRotationMatrix, mInclinationMatrix,
          mAccelerometerReading, mMagnetometerReading);
      // "mRotationMatrix" now has up-to-date information.

      SensorManager.getOrientation(mRotationMatrix, mOrientationAngles);
      // "mOrientationAngles" now has up-to-date information.

      SensorManager.getQuaternionFromVector(mOrientationQuaternion,  mOrientationAngles);
      // "mOrientationQuaternion" now has up-to-date information.

      inclination = SensorManager.getInclination(mInclinationMatrix);
      // "inclination" now has up-to-date information.
  }

  public void updateOrientationAnglesFromVector() {
      // Update rotation matrix, which is needed to update orientation angles.
      SensorManager.getRotationMatrixFromVector(mRotationMatrix, mRotationVectorReading);
      // "mRotationMatrix" now has up-to-date information.

      SensorManager.getOrientation(mRotationMatrix, mOrientationAngles);
      // "mOrientationAngles" now has up-to-date information.

      SensorManager.getQuaternionFromVector(mOrientationQuaternion,  mRotationVectorReading);
      // "mOrientationQuaternion" now has up-to-date information.

      SensorManager.getQuaternionFromVector(mOrientationQuaternion_device,  mGameRotationVectorReading);
      // "mOrientationQuaternion_device" now has up-to-date information.
      //
      inclination = null;
      // "inclination" is null since using rotation vector
  }

  public float[] getRotation(){
   return mRotationMatrix;
  }

  public float[] getOrientation(){
   return mOrientationAngles;
  }

  public void sendWorldImuMsgsToRos(){
    if (publisherWorld != null) {
      sensor_msgs.Imu imu_msg = (sensor_msgs.Imu)publisherWorld.newMessage();

      imu_msg.getOrientation().setW(mOrientationQuaternion[0]);
      imu_msg.getOrientation().setX(mOrientationQuaternion[1]);
      imu_msg.getOrientation().setY(mOrientationQuaternion[2]);
      imu_msg.getOrientation().setZ(mOrientationQuaternion[3]);
      imu_msg.setOrientationCovariance(orientationCovariance);

      imu_msg.getAngularVelocity().setX(mGyroscopeReading[0]);
      imu_msg.getAngularVelocity().setY(mGyroscopeReading[1]);
      imu_msg.getAngularVelocity().setZ(mGyroscopeReading[2]);
      imu_msg.setAngularVelocityCovariance(angularCovariance);

      imu_msg.getLinearAcceleration().setX(mLinearAccelerationReading[0]);
      imu_msg.getLinearAcceleration().setY(mLinearAccelerationReading[1]);
      imu_msg.getLinearAcceleration().setZ(mLinearAccelerationReading[2]);
      imu_msg.setLinearAccelerationCovariance(linearCovariance);

      imu_msg.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
      imu_msg.getHeader().setFrameId("world");
      publisherWorld.publish(imu_msg);
    }
  }

  public void sendDeviceImuMsgsToRos(){
    if (publisherDevice != null) {
      sensor_msgs.Imu imu_msg = (sensor_msgs.Imu)publisherDevice.newMessage();

      imu_msg.getOrientation().setW(mOrientationQuaternion_device[0]);
      imu_msg.getOrientation().setX(mOrientationQuaternion_device[1]);
      imu_msg.getOrientation().setY(mOrientationQuaternion_device[2]);
      imu_msg.getOrientation().setZ(mOrientationQuaternion_device[3]);
      imu_msg.setOrientationCovariance(orientationCovariance);

      imu_msg.getAngularVelocity().setX(mGyroscopeReading[0]);
      imu_msg.getAngularVelocity().setY(mGyroscopeReading[1]);
      imu_msg.getAngularVelocity().setZ(mGyroscopeReading[2]);
      imu_msg.setAngularVelocityCovariance(angularCovariance);

      imu_msg.getLinearAcceleration().setX(mLinearAccelerationReading[0]);
      imu_msg.getLinearAcceleration().setY(mLinearAccelerationReading[1]);
      imu_msg.getLinearAcceleration().setZ(mLinearAccelerationReading[2]);
      imu_msg.setLinearAccelerationCovariance(linearCovariance);

      imu_msg.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
      imu_msg.getHeader().setFrameId("android");
      publisherDevice.publish(imu_msg);
    }
  }

  public void sendWorldVector3StampedMsgsToRos(){
    if (publisherWorldVector3Stamped != null) {
      geometry_msgs.Vector3Stamped v3s_msg = (geometry_msgs.Vector3Stamped)publisherWorldVector3Stamped.newMessage();

      v3s_msg.getVector().setX((double) mRotationVectorReading[0]);
      v3s_msg.getVector().setY((double) mRotationVectorReading[1]);
      v3s_msg.getVector().setZ((double) mRotationVectorReading[2]);

      v3s_msg.getHeader().setStamp(Time.fromMillis(System.currentTimeMillis()));
      v3s_msg.getHeader().setFrameId("world");
      publisherWorldVector3Stamped.publish(v3s_msg);
    }
  }

  public String getMessage(){
      StringBuilder builder = new StringBuilder();

      builder.append("\n\n");
      builder.append("Sensor Accuracy Level: ");
      builder.append("\n");
      builder.append("     Accelerometer: ");
      builder.append(accelerometerAccuracyLevel);
      builder.append("\n");
      builder.append("     Magnetic Field: ");
      builder.append(magneticFieldAccuracyLevel);
      builder.append("\n");
      builder.append("     Rotation Vector: ");
      builder.append(rotationVectorAccuracyLevel);
      builder.append("\n");
      builder.append("     Linear Acceleration: ");
      builder.append(linearAccelerationAccuracyLevel);
      builder.append("\n");
      builder.append("     Gyroscope: ");
      builder.append(gyroscopeAccuracyLevel);

      builder.append("\n\n");
      builder.append("Orientation Angle [rad]: ");
      builder.append("\n");
      builder.append("     Azimuth: ");
      builder.append(mOrientationAngles[0]);
      builder.append("\n");
      builder.append("     Pitch: ");
      builder.append(mOrientationAngles[1]);
      builder.append("\n");
      builder.append("     Roll: ");
      builder.append(mOrientationAngles[2]);
      //builder.append("\nRotation Matrix: ");
      //for (float value : mRotationMatrix) {
      //    if (builder.length() > 0) {
      //        builder.append(" | ");
      //    }
      //    builder.append(value);
      //}
      builder.append("\n\n");
      builder.append("Linear Acceleration [m/s/s] (excluding gravity): ");
      builder.append("\n");
      builder.append("     X: ");
      builder.append(mLinearAccelerationReading[0]);
      builder.append("\n");
      builder.append("     Y: ");
      builder.append(mLinearAccelerationReading[1]);
      builder.append("\n");
      builder.append("     Z: ");
      builder.append(mLinearAccelerationReading[2]);

      builder.append("\n\n");
      builder.append("Gyroscope [rad/s]: ");
      builder.append("\n");
      builder.append("     X: ");
      builder.append(mGyroscopeReading[0]);
      builder.append("\n");
      builder.append("     Y: ");
      builder.append(mGyroscopeReading[1]);
      builder.append("\n");
      builder.append("     Z: ");
      builder.append(mGyroscopeReading[2]);

      if( inclination != null) {
         builder.append("\n\n");
         builder.append("Inclination [rad]: ");
         builder.append(inclination);
      }

    return builder.toString();
  }
  
  @Override
  public void onShutdown(Node node) {
    // shutdown ROS publisher
    publisherWorld.shutdown();
    publisherDevice.shutdown();
    publisherWorldVector3Stamped.shutdown();
    // shutdown android sensors
    mSensorManager.unregisterListener(this);
  }
  
  @Override        
  public void onShutdownComplete(Node node) {
      publisherWorld = null;
      publisherDevice = null;
      publisherWorldVector3Stamped = null;
  }

  // Low pass filer for sensor readings
  // 0 <= ALPHA <= 1; with smaller ALPHA == more smoothing effect
  public float[] lowPass( float[] input, float[] output ) {
     if ( output == null ) return input;     
        for ( int i=0; i<input.length; i++ ) {
           output[i] = output[i] + ALPHA * (input[i] - output[i]);
        }
     return output;
  }

}
