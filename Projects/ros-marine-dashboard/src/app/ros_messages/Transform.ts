import { RosMessage } from './RosMessage';
import { Vector3 } from './Vector3';
import { Quaternion } from './Quaternion';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Transform implements RosMessage {

  public type = 'geometry_msgs/Transform';

  public translation: Vector3;
  public rotation: Quaternion;

  constructor() {}

  public dataSub = new BehaviorSubject<Object>(this);

  updateData(data: any) {
    this.translation = data.Vector3 as Vector3;
    this.rotation = data.rotation as Quaternion;
    this.dataSub.next(this);
  }

}