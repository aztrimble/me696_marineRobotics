import { Injectable } from '@angular/core';
import * as ROSLIB from 'roslib';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

import * as RosMessages from '../ros_messages/index';


@Injectable()
export class RoslibService {

  private rbServer: ROSLIB.Ros;
  private availableTopics = [];
  public availableTopicsSub = new BehaviorSubject<Array<Object>>([]);
  private unavailableTopics = [];
  public unavailableTopicsSub = new BehaviorSubject<Array<Object>>([]);
  private roscoreIP = '192.168.1.28';
  private tileserverIP = '192.168.1.117';
  public useTileServer = false;

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() { }

  public setServerInfo(rosIP: string, tileserverIP: string, useTileServer: boolean) {
    this.roscoreIP = rosIP;
    this.tileserverIP = tileserverIP;
    this.useTileServer = useTileServer;
  }

  public connect(): Promise<string> {
    return new Promise((resolve, reject) => {
      if (this.rbServer) {
        this.rbServer.close();
      }
      this.rbServer = new ROSLIB.Ros({
        url: `ws://${this.roscoreIP}:9090`
      });
      this.rbServer.on('connection', () => {
        this.updateTopics();
        return resolve('Connected to ROSCORE');
      });
      // This function is called when there is an error attempting to connect to rosbridge
      this.rbServer.on('error', (error) => {
        return resolve(`Error connecting to websocket server. ${error.toString()}`);
      });
      // This function is called when the connection to rosbridge is closed
      this.rbServer.on('close', () => {
        return resolve('Connection to websocket server closed');
      });
    });
  }

  public disconnect() {
    this.rbServer.close();
  }

  public getRosCoreIp() {
    return this.roscoreIP;
  }

  public getTileServerIp() {
    return this.tileserverIP;
  }

  public createTopic(topicName: string, messageType: string): ROSLIB.Topic {
    const topic = new ROSLIB.Topic({
      ros: this.rbServer,
      name: topicName,
      messageType: messageType,
      queue: 1
    });
    return topic;
  }

  public updateTopics() {
    const topicsClient = new ROSLIB.Service({
      ros: this.rbServer,
      name: '/rosapi/topics',
      serviceType: '/rosapi/Topics'
    });
    this.availableTopics = [];
    this.unavailableTopics = [];
    const request = new ROSLIB.ServiceRequest();
    topicsClient.callService(request, (result) => {
      const promises = [];
      result.topics.forEach(topic => {
        promises.push(this.getTopicInfo(topic));
      });
      Promise.all(promises).then(topics => {
        topics.forEach(el => {
          if (el.object != undefined) {
            this.availableTopics.push(el);
          } else {
            this.unavailableTopics.push(el);
          }
        })
        console.log(this.availableTopics, this.unavailableTopics);
        this.availableTopicsSub.next(this.availableTopics);
        this.unavailableTopicsSub.next(this.unavailableTopics);
      })
    });
  }

  private getTopicInfo(topicName: string): Promise<Object> {
    const topicsClient = new ROSLIB.Service({
      ros: this.rbServer,
      name: '/rosapi/topic_type',
      serviceType: '/rosapi/Topics'
    });
    const request = new ROSLIB.ServiceRequest({ topic: topicName });
    return new Promise((resolve, reject) => topicsClient.callService(request, (result) => {
      const object = RosMessages.GetRosMessageObject(result.type);
      if (object) {
        this.createTopic(topicName, result.type).subscribe((data: any) => {
          object.updateData(data);
        });
      }
      const topic = new Topic();
      topic.name = topicName;
      topic.type = result.type;
      topic.object = object;
      return resolve({ 'name': topicName, 'type': result.type, 'object': object });


    }));
  }

  public getTopicsByType(topicType: string) {
    const topics = [];
    this.availableTopics.forEach(topic => {
      if (topic.type === `/${topicType}`) {
        topics.push(topic);
      }
    });
    return topics;
  }

}

export class Topic {
  name: string;
  type: string;
  object: any;
}
