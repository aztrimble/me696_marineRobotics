import { RosMessage } from './RosMessage';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class PointField implements RosMessage {

  public type = 'sensor_msgs/PointField';

  public name: string;
  public offset: number;
  public datatype: number;
  public count: number;

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  getDataType(): String {
    return DataType[this.datatype];
  }

  updateData(data: any) {
    this.name = data.name;
    this.offset = data.offset;
    this.datatype = data.datatype as DataType;
    this.count = data.count;
    this.dataSub.next(this);
  }
}

enum DataType {
  INT8 = 1,
  UINT8 = 2,
  INT16 = 3,
  UINT16 = 4,
  INT32 = 5,
  UINT32 = 6,
  FLOAT32 = 7,
  FLOAT64 = 8
}