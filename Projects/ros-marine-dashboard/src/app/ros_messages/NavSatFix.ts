import { RosMessage } from './RosMessage';
import { Header } from './Header';
import { NavSatStatus } from './NavSatStatus';
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class NavSatFix implements RosMessage {

  public type = 'sensor_msgs/NavSatFix';

  public header: Header;
  public status: NavSatStatus
  public latitude: number;
  public longitude: number;
  public altitude: number;
  public position_covariance: number[];
  public position_covariance_type: number;
  
  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  getPositionCovarianceType(): String {
    return PositionCovarianceType[this.position_covariance_type];
  }
  
  updateData(data: any) {
    this.header = data.header as Header;
    this.status = data.status as NavSatStatus;
    this.latitude = data.latitude;
    this.longitude = data.longitude;
    this.altitude = data.altitude;
    this.position_covariance = data.position_covariance;
    this.position_covariance_type = data.position_covariance_type;
    this.dataSub.next(this);
  }

}

enum PositionCovarianceType {
  COVARIANCE_TYPE_UNKNOWN = 0,
  COVARIANCE_TYPE_APPROXIMATED = 1,
  COVARIANCE_TYPE_DIAGONAL_KNOWN = 2,
  COVARIANCE_TYPE_KNOWN = 3
}
