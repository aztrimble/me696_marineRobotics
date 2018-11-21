package org.ros.android.android_kanaloa_camera_one;

import android.hardware.Camera.Size;

interface KanaloaRawImageListener {

  void onNewRawImage(byte[] data, Size size);

}