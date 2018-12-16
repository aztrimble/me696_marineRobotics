import { RosMessage } from "./RosMessage";
import { BehaviorSubject } from "rxjs/BehaviorSubject";

export class Float64 implements RosMessage {

  public type = 'std_msgs/Float64';

  public data: number;

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  updateData(data: any) {
    this.data = data.data;
    this.dataSub.next(this);
  }

}
