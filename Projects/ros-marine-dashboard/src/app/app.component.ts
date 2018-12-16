import { Component } from '@angular/core';
import { RoslibService } from './roslib/roslib.service';

@Component({
  selector: 'app-root',
  templateUrl: './app.component.html',
  styleUrls: ['./app.component.css']
})
export class AppComponent {

  roscoreIp = '192.168.1.28';
  //roscoreIp = 'rip.eng.hawaii.edu';
  tileserverIp = '192.168.99.100'
  useTileServer = false;
  loading = true;

  constructor(private rosLib: RoslibService) {}

  setIP() {
    if (this.roscoreIp && this.tileserverIp) {
      this.loading = true;
      this.rosLib.setServerInfo(this.roscoreIp, this.tileserverIp, this.useTileServer);
      this.rosLib.connect().then((result) => {
        console.log(result);
        if (result == 'Connected to ROSCORE') {
          this.loading = false;
        }
      });
    }
  }

  disconnect() {
    this.rosLib.disconnect();
    this.loading = true;
  }

  refreshTopics() {
    this.rosLib.updateTopics();
  }
}

