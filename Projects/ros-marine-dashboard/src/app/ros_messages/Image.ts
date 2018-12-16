import { RosMessage } from './RosMessage';
import { Header } from './Header';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Image implements RosMessage {

  public type = 'sensor_msgs/Image';

  public header: Header;
  public height: number;
  public width: number;
  public encoding: string;
  public is_bigendian: number;
  public step: number;
  public data: number[];

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  updateData(data: any) {
    this.header = data.header as Header;
    this.height = data.height;
    this.width = data.width;
    this.encoding = data.encoding;
    this.is_bigendian = data.is_bigendian;
    this.step = data.step;
    this.data = data.data;
    this.dataSub.next(this);
  }

}