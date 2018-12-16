import { Component, AfterViewInit } from '@angular/core';
declare let L;
import 'leaflet-rotatedmarker';
import * as THREE from 'three';

import { RoslibService, Topic } from '../roslib/roslib.service';
import { NavSatFix, Vector3Stamped } from '../ros_messages/index';
import { Subscription } from 'rxjs/Subscription';


@Component({
  selector: 'app-map',
  templateUrl: './map.component.html',
  styleUrls: ['./map.component.css']
})
export class MapComponent implements AfterViewInit {

  THREE: any;
  lockGPS = true;
  map: any;
  boatIcon: any;
  boatMarker = null;
  lastLatLong = null;
  heading = null;

  navSatFixTopics = [];
  navSatFixTopic: Topic;
  navSatFixDataSub: Subscription;
  navSatFix = new NavSatFix();

  imuTopics = [];
  imuTopic: Topic;
  imuDataSub: Subscription;
  imu = new Vector3Stamped();

  constructor(private rosLib: RoslibService) {
    this.boatIcon = L.icon({
      iconUrl: './assets/wamv-icon.png',
      iconSize: [40, 20], // size of the icon
      iconAnchor: [20, 10], // point of the icon which will correspond to marker's location
    });
  }

  ngAfterViewInit() {
    this.THREE = THREE;
    this.rosLib.availableTopicsSub.subscribe((data) => {
      this.updateTopics();
    })
    this.createMap();
  }

  updateTopics() {
    this.navSatFixTopics = this.rosLib.getTopicsByType(this.navSatFix.type);
    if (this.navSatFixTopic) {
      const stillAvailable = this.navSatFixTopics.find(el => el.name == this.navSatFixTopic.name);
      if (!stillAvailable) this.disconnectGPS();
    }
    this.imuTopics = this.rosLib.getTopicsByType(this.imu.type);
    if (this.imuTopic) {
      const stillAvailable = this.imuTopics.find(el => el.name == this.imuTopic.name);
      if (!stillAvailable) this.disconnectIMU();
    }
  }

  changeNavSatFixTopic(topic) {
    if (this.navSatFixDataSub) this.navSatFixDataSub.unsubscribe();
    this.navSatFixTopic = topic.value;
    this.navSatFix = this.navSatFixTopic.object as NavSatFix;
    this.navSatFixDataSub = this.navSatFix.dataSub.subscribe(data => {
      this.addMarker(this.navSatFix.latitude, this.navSatFix.longitude);
    });
  }

  changeImuTopic(topic) {
    if (this.imuDataSub) this.imuDataSub.unsubscribe();
    this.imuTopic = topic.value;
    this.imu = this.imuTopic.object as Vector3Stamped;
    this.imuDataSub = this.imu.dataSub.subscribe(data => {
      this.heading = this.imu.vector.y * (180 / Math.PI) + 90 - 9;
    });
  }

  createMap() {
    this.map = L.map('map', { attributionControl: false }).setView([21.296894, -157.778955], 12);
    if (this.rosLib.useTileServer) {
      L.tileLayer(`http://${this.rosLib.getTileServerIp()}:8080/styles/osm-bright/{z}/{x}/{y}.png`).addTo(this.map);
    } else {
      L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
        attribution: 'Tiles &copy; Esri &mdash; Source: Esri, i-cubed, USDA, USGS, AEX, GeoEye, Getmapping, Aerogrid, IGN, IGP, UPR-EGP, and the GIS User Community'
      }).addTo(this.map);
    }
    this.map.invalidateSize();
  }

  addMarker(lat: number, long: number) {
    if (lat && long) {
      this.lastLatLong = L.latLng(lat, long);
      L.circle(this.lastLatLong, {
        color: 'red',
        fillColor: 'red',
        fillOpacity: 0.5,
        radius: 2
      }).addTo(this.map);
    }

    if (this.boatMarker) {
      this.lastLatLong = L.latLng(lat, long);
      this.map.removeLayer(this.boatMarker);
      this.boatMarker = L.marker(this.lastLatLong, { icon: this.boatIcon, rotationAngle: this.heading }).addTo(this.map);
      this.boatMarker.setRotationAngle(this.heading);
    } else {
      this.lastLatLong = L.latLng(lat, long);
      this.boatMarker = L.marker(this.lastLatLong, { icon: this.boatIcon }).addTo(this.map);
    }
    if (this.lockGPS) {
      this.map.setView(L.latLng(lat, long));
    }
    this.map.invalidateSize();
  }

  disconnectGPS() {
    if (this.navSatFixDataSub) {
      this.navSatFixDataSub.unsubscribe();
    }
    this.navSatFixTopic = null;
    this.navSatFix = new NavSatFix();;
  }

  disconnectIMU() {
    if (this.imuDataSub) {
      this.imuDataSub.unsubscribe();
    }
    this.imuTopic = null;
    this.imu = new Vector3Stamped();;
  }

  lockUnlockGPS() {
    this.lockGPS = !this.lockGPS;
  }

}
