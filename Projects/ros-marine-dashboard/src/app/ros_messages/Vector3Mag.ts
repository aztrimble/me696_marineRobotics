import { RosMessage } from './RosMessage';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Vector3Mag implements RosMessage {

  public type = 'geometry_mags/Vector3Stamped';

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