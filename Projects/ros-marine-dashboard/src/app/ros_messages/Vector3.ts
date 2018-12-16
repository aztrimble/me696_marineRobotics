import { RosMessage } from './RosMessage';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Vector3 implements RosMessage {

  public type = 'geometry_msgs/Vector3';

  public x: number;
  public y: number;
  public z: number;

  constructor() {}

  public dataSub = new BehaviorSubject<Object>(this);

  updateData(data: any) {
    this.x = data.x;
    this.y = data.y;
    this.z = data.z;
    this.dataSub.next(this);
  }

}