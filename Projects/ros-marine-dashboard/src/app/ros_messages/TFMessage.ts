import { RosMessage } from './RosMessage';
import { TransformStamped } from './TransformStamped';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class TFMessage implements RosMessage {

  public type = 'tf2_msgs/TFMessage'

  public transforms: TransformStamped[];

  constructor() {}

  public dataSub = new BehaviorSubject<Object>(this);

  updateData(data) {
    this.transforms = data.transforms;
    this.dataSub.next(this);
  }

}