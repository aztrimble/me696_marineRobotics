import { RosMessage } from './RosMessage';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class CompressedImage implements RosMessage {

  public type = 'sensor_msgs/CompressedImage';

  public format: string;
  public data: number[];

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  getImage() {
    return `data:image/jpeg;base64,${this.data}`;
  }

  updateData(data: any) {
    this.format = data.format;
    this.data = data.data;
    this.dataSub.next(this);
  }
  
}