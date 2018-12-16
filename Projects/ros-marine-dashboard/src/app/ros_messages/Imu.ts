import { RosMessage } from './RosMessage';
import { Header } from "./Header";
import { Quaternion } from "./Quaternion";
import { Vector3 } from "./Vector3";
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Imu implements RosMessage {

  public type = 'sensor_msgs/Imu';

  public header: Header;
  public orientation: Quaternion
  public orientation_covariance: number[];
  public angular_velocity: Vector3;
  public angular_velocity_covariance: number[];
  public linear_acceleration: Vector3;
  public linear_acceleration_covariance: number[];

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  updateData(data: any) {
    this.header = data.header as Header;
    this.orientation = data.orientation;
    this.orientation_covariance = data.orientation_covariance;
    this.angular_velocity = data.angular_velocity;
    this.angular_velocity_covariance = data.angular_velocity_covariance;
    this.linear_acceleration = data.linear_acceleration;
    this.linear_acceleration_covariance = data.linear_acceleration_covariance;
    this.dataSub.next(this);
  }

}
