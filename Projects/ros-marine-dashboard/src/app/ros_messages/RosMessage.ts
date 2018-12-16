import { BehaviorSubject } from "rxjs/BehaviorSubject";

export interface RosMessage {
  type: string;
  dataSub: BehaviorSubject<Object>;
  updateData(data: any);

}