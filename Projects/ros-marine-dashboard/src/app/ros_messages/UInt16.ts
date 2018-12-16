import { RosMessage } from './RosMessage';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class UInt16 implements RosMessage {

  public type = 'std_msgs/UInt16';

  public data: number;

  constructor() {}

  public dataSub = new BehaviorSubject<Object>(this);

  updateData(data: any) {
    this.data = data.data;
    this.dataSub.next(this);
  }

}
