import { RosMessage } from './RosMessage';
import { Header } from "./Header";
import { PointField } from "./PointField";
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class PointCloud2 implements RosMessage {

  public type = 'sensor_msgs/PointCloud2';

  public header: Header
  public height: number;
  public width: number;
  public fields: PointField[];
  public is_bigendian: boolean;
  public point_step: number;
  public row_step: number;
  public data: number[];
  public is_dense: boolean;

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  updateData(data: any) {
    this.header = data.header as Header;
    this.height = data.height;
    this.width = data.width;
    this.fields = data.fields;
    this.is_bigendian = data.is_bigendian;
    this.point_step = data.point_step;
    this.row_step = data.row_step;
    this.data = data.data;
    this.is_dense = data.is_dense;
    this.dataSub.next(this);
  }

}