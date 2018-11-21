/*
 * Copyright (C) 2011 Google Inc.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package org.ros.android.android_kanaloa_location;

import android.content.Context;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Build;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.Window;
import android.view.WindowManager;
import android.widget.TextView;
import android.widget.Toast;
import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;
import android.util.Log;
import java.io.IOException;

/**
 * @author ethan.rublee@gmail.com (Ethan Rublee)
 * @author damonkohler@google.com (Damon Kohler)
 */
public class MainActivity extends RosActivity {


  private AndroidPhoneImuNode kalanoaAndroidImuNode;
  private AndroidPhoneGpsNode kalanoaAndroidGpsNode;
  private LocationManager locationManager;
  private LocationListener locationListener;

  public MainActivity() {
    super("Kanaloa Location", "Kanaloa Location");
  }

  @Override
  protected void onCreate(Bundle savedInstanceState) {
    super.onCreate(savedInstanceState);
    //requestWindowFeature(Window.FEATURE_NO_TITLE);
    //getWindow().addFlags(WindowManager.LayoutParams.FLAG_FULLSCREEN);
    setContentView(R.layout.main);
    kalanoaAndroidImuNode = new AndroidPhoneImuNode(this);
    kalanoaAndroidGpsNode = new AndroidPhoneGpsNode(this);
    // init location listener
    locationManager = (LocationManager) getSystemService(Context.LOCATION_SERVICE);
    this.locationListener = new LocationListener() {
        @Override
        public void onLocationChanged(Location loc) {
          kalanoaAndroidGpsNode.updateLocationInfo(loc);
        }
        @Override
        public void onProviderDisabled(String provider) {}
        @Override
        public void onProviderEnabled(String provider) {}
        @Override
        public void onStatusChanged(String provider, int status, Bundle extras) {}
      };
    locationManager.requestLocationUpdates(LocationManager.GPS_PROVIDER, 500, 0, locationListener);
    locationManager.requestLocationUpdates(LocationManager.NETWORK_PROVIDER, 500, 0, locationListener);

    // get last known location info
    Location loc = locationManager.getLastKnownLocation(LocationManager.GPS_PROVIDER);
    if(loc == null) {
      loc = locationManager.getLastKnownLocation(LocationManager.NETWORK_PROVIDER);
    }
    kalanoaAndroidGpsNode.updateLocationInfo(loc);
  }


  @Override
  protected void onResume() {
      super.onResume();
  }

  @Override
  protected void onPause() {
      super.onPause();

      // Don't receive any more updates from either sensor.
      //mSensorManager.unregisterListener(this);
  }

  @Override
  protected void onDestroy() {
      super.onDestroy();
      // stop receiving GPS updates
      locationManager.removeUpdates(locationListener);
  }

  @Override
  public boolean onTouchEvent(MotionEvent event) {
    if (event.getAction() == MotionEvent.ACTION_UP) {
      final Toast toast;
      toast = Toast.makeText(this, "updated information", Toast.LENGTH_SHORT);


      TextView textView = (TextView) findViewById(R.id.textView2);
      textView.setText("Touch screen to view current sensor information" +
                        kalanoaAndroidImuNode.getMessage() +
                        kalanoaAndroidGpsNode.getMessage());

      runOnUiThread(new Runnable() {
        @Override
        public void run() {
          toast.show();
        }
      });
    }
    return true;
  }

  @Override
  protected void init(NodeMainExecutor nodeMainExecutor) {

    try {
      java.net.Socket socket = new java.net.Socket(getMasterUri().getHost(), getMasterUri().getPort());
      java.net.InetAddress local_network_address = socket.getLocalAddress();
      socket.close();
      NodeConfiguration nodeConfiguration =
              //NodeConfiguration.newPublic(local_network_address.getHostAddress(), getMasterUri());
              NodeConfiguration.newPublic(getRosHostname(), getMasterUri());
      nodeMainExecutor.execute(kalanoaAndroidImuNode, nodeConfiguration);
      nodeMainExecutor.execute(kalanoaAndroidGpsNode, nodeConfiguration);
    } catch (IOException e) {
      // Socket problem
      Log.e("Kanaloa Location", "socket error trying to get networking information from the master uri");
    }

  }

}
