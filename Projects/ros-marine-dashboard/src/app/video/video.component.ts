import { Component, OnInit, ViewChild, ChangeDetectorRef } from '@angular/core';
import { RoslibService, Topic } from '../roslib/roslib.service';
import { CompressedImage } from '../ros_messages/index';
import { Subscription } from 'rxjs/Subscription';

@Component({
  selector: 'app-video',
  templateUrl: './video.component.html',
  styleUrls: ['./video.component.css']
})
export class VideoComponent implements OnInit {

  loaded = false;
  videoTopics = [];
  private topic: Topic;
  private dataSub: Subscription;

  @ViewChild('video') videoData;
  private compressedImage = new CompressedImage();

  constructor(private rosLib: RoslibService, private changeDetectorRef: ChangeDetectorRef) { }

  ngOnInit() {
    this.rosLib.availableTopicsSub.subscribe((data) => {
      this.updateTopics();
    })
  }

  updateTopics() {
    this.videoTopics = this.rosLib.getTopicsByType(this.compressedImage.type);
    if (this.topic) {
      const stillAvailable = this.videoTopics.find(el => el.name == this.topic.name);
      if (!stillAvailable) this.disconnectVideo();
    }
  }

  public changeTopic(topic) {
    if (this.dataSub) this.dataSub.unsubscribe();
    this.topic = topic.value;
    this.compressedImage = this.topic.object as CompressedImage;
    this.loaded = true;
    this.changeDetectorRef.detectChanges();
    this.dataSub = this.compressedImage.dataSub.subscribe(data => {
      this.updateImage();
    });
  }

  public disconnectVideo() {
    this.loaded = false;
    if (this.dataSub) {
      this.dataSub.unsubscribe();
    }
    this.topic = null;
    this.compressedImage = new CompressedImage();;
  }

  private updateImage() {
    this.videoData.nativeElement.src = this.compressedImage.getImage();
  }


}
