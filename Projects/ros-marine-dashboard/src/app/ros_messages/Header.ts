import { RosMessage } from './RosMessage';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Header implements RosMessage {

  public type: 'std_msgs/Header';

  public seq: number;
  public time: any;
  public frame_id: string;

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  updateData(data: any) {
    this.seq = data.seq;
    this.time = data.time;
    this.frame_id = data.frame_id;
    this.dataSub.next(this);
  }

}
