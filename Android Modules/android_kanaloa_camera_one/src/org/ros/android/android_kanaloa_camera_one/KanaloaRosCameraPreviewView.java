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

package org.ros.android.android_kanaloa_camera_one;

import android.content.Context;
import android.util.AttributeSet;

import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;

/**
 * Displays and publishes preview frames from the camera.
 * 
 * @author damonkohler@google.com (Damon Kohler)
 */
public class KanaloaRosCameraPreviewView extends KanaloaCameraPreviewView implements NodeMain {

  private String nodeName = "kanaloa/android_default";

  public KanaloaRosCameraPreviewView(Context context) {
    super(context);
  }

  public KanaloaRosCameraPreviewView(Context context, AttributeSet attrs) {
    super(context, attrs);
  }

  public KanaloaRosCameraPreviewView(Context context, AttributeSet attrs, int defStyle) {
    super(context, attrs, defStyle);
  }

  public void setNodeName (String name){
    nodeName = name;
  }

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of(nodeName);
  }

  @Override
  public void onStart(ConnectedNode connectedNode) {
    setRawImageListener(new KanaloaCompressedImagePublisher(connectedNode));
  }

  @Override
  public void onShutdown(Node node) {
  }

  @Override
  public void onShutdownComplete(Node node) {
  }

  @Override
  public void onError(Node node, Throwable throwable) {
  }
}
