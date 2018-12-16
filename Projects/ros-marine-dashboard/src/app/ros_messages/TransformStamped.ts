import { RosMessage } from './RosMessage';
import { Header } from './Header';
import { Transform } from './Transform';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class TransformStamped implements RosMessage {

  public type = 'geometry_msgs/TransformStamped';

  public header: Header;
  public child_frame_id: string;
  public transform: Transform;

  constructor() {}

  public dataSub = new BehaviorSubject<Object>(this);

  updateData(data: any) {
    this.header = data.header as Header;
    this.child_frame_id = data.child_frame_id;
    this.transform = data.transform as Transform;
    this.dataSub.next(this);
  }

}