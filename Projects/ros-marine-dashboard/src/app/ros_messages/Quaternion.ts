import { RosMessage } from './RosMessage';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Quaternion implements RosMessage {

  public type = 'geometry_msgs/Quaternion';

  public x: number;
  public y: number;
  public z: number;
  public w: number;

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  updateData(data: any) {
    this.x = data.x;
    this.y = data.y;
    this.z = data.z;
    this.w = data.w;
    this.dataSub.next(this);
  }

}