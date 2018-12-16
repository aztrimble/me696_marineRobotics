import { RosMessage } from './RosMessage';
import { Header } from "./Header";
import { BehaviorSubject } from 'rxjs/BehaviorSubject';
import { Vector3 } from './Vector3';

export class Vector3Stamped implements RosMessage {

  public type = 'geometry_msgs/Vector3Stamped';

  public header: Header
  public child_frame_id: string;
  public vector: Vector3;

  constructor() {}

  public dataSub = new BehaviorSubject<Object>(this);

  updateData(data: any) {
    this.header = data.header as Header;
    this.child_frame_id = data.child_frame_id;
    this.vector = data.vector as Vector3;
    this.dataSub.next(this);
  }

}