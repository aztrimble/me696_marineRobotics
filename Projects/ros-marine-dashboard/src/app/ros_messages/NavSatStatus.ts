import { RosMessage } from './RosMessage';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class NavSatStatus implements RosMessage {

  public type = 'sensor_msgs/NavSatStatus';

  public status: number;
  public service: number;

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  getStatus(): String {
    return Status[this.status];
  }

  getService(): String {
    return Service[this.service];
  }

  updateData(data: any) {
    this.status = data.status as Status;
    this.service = data.service as Service;
    this.dataSub.next(this);
  }

}

enum Status {
  STATUS_NO_FIX = -1,
  STATUS_FIX = 0,
  STATUS_SBAS_FIX = 1,
  STATUS_GBAS_FIX = 2
}

enum Service {
  SERVICE_GPS = 1,
  SERVICE_GLONASS = 2,
  SERVICE_COMPASS = 4,
  SERVICE_GALILEO = 8
}
