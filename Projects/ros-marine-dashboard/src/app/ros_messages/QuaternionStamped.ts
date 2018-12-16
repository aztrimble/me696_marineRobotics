import { RosMessage } from './RosMessage';
import { Header } from "./Header";
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class QuaternionStamped implements RosMessage {

  public type = 'geometry_msgs/QuaternionStamped';

  public header: Header;
  public x: number;
  public y: number;
  public z: number;
  public w: number;

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  updateData(data: any) {
    this.header = data.header as Header;
    this.x = data.x;
    this.y = data.y;
    this.z = data.z;
    this.w = data.w;
    this.dataSub.next(this);
  }

}