import { RosMessage } from './RosMessage';
import { Vector3 } from "./Vector3";
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Twist implements RosMessage {

  public type: 'geometry_msgs/Twist';

  public linear: Vector3;
  public angular: Vector3;

  constructor() {}

  public dataSub = new BehaviorSubject<Object>(this);

  updateData(data: any) {
    this.linear = data.linear as Vector3;
    this.angular = data.angular as Vector3;
    this.dataSub.next(this);
  }

}