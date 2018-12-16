import { RosMessage } from './RosMessage';
import { Vector3 } from "./Vector3";
import { Header } from "./Header";
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class TwistStamped implements RosMessage {

  public type: 'geometry_msgs/TwistStamped';

  public header: Header
  public child_frame_id: string;
  public linear: Vector3;
  public angular: Vector3;

  constructor() {}

  public dataSub = new BehaviorSubject<Object>(this);

  updateData(data: any) {
    this.header = data.header as Header;
    this.child_frame_id = data.child_frame_id;
    this.linear = data.linear as Vector3;
    this.angular = data.angular as Vector3;
    this.dataSub.next(this);
  }

}