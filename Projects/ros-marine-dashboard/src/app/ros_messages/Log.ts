import { RosMessage } from './RosMessage';
import { Header } from "./Header";
import { BehaviorSubject } from 'rxjs/BehaviorSubject';

export class Log implements RosMessage {

  public type = 'rosgraph_msgs/Log';

  public header: Header;
  public level: number;
  public name: string;
  public msg: string;
  public file: string;
  public function: string;
  public line: number;
  public topics: string[];

  public dataSub = new BehaviorSubject<Object>(this);

  constructor() {}

  getLevel(): String {
    return Type[this.level];
  }

  updateData(data: any) {
    this.header = data.header as Header;
    this.level = data.level;
    this.name = data.name;
    this.msg = data.msg;
    this.file = data.file;
    this.function = data.function;
    this.line = data.line;
    this.topics = data.topics;
    this.dataSub.next(this);
  }

}

enum Type {
  DEBUG = 1,
  INFO = 2,
  WARN = 4,
  ERROR = 8,
  FATAL = 16
}