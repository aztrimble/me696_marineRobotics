import { CompressedImage } from './CompressedImage';
import { Float32 } from './Float32';
import { Float64 } from './Float64';
import { Header } from './Header';
import { Image } from './Image';
import { Imu } from './Imu';
import { Log } from './Log';
import { NavSatFix } from './NavSatFix';
import { NavSatStatus } from './NavSatStatus';
import { PointCloud2 } from './PointCloud2';
import { PointField } from './PointField';
import { Quaternion } from './Quaternion';
import { QuaternionStamped } from './QuaternionStamped';
import { TFMessage } from './TFMessage';
import { Transform } from './Transform';
import { TransformStamped } from './TransformStamped';
import { Twist } from './Twist';
import { TwistStamped } from './TwistStamped';
import { UInt16 } from './UInt16';
import { Vector3 } from './Vector3';
import { Vector3Mag } from './Vector3Mag';
import { Vector3Stamped } from './Vector3Stamped';

export * from './CompressedImage';
export * from './Float32';
export * from './Float64';
export * from './Header';
export * from './Image';
export * from './Imu';
export * from './Log';
export * from './NavSatFix';
export * from './NavSatStatus';
export * from './PointCloud2';
export * from './PointField';
export * from './Quaternion';
export * from './QuaternionStamped';
export * from './TFMessage';
export * from './Transform';
export * from './TransformStamped';
export * from './Twist';
export * from './TwistStamped';
export * from './UInt16';
export * from './Vector3';
export * from './Vector3Mag';
export * from './Vector3Stamped';

const compressedImage = new CompressedImage();
const float32 = new Float32();
const float64 = new Float64();
const header = new Header();
const image = new Image();
const imu = new Imu();
const log = new Log();
const navSatFix = new NavSatFix();
const navSatStatus = new NavSatStatus();
const pointCloud2 = new PointCloud2();
const pointField = new PointField();
const quaternion = new Quaternion();
const quaternionStamped = new QuaternionStamped();
const tFMessage = new TFMessage();
const transform = new Transform();
const transformStamped = new TransformStamped();
const twist = new Twist();
const twistStamped = new TwistStamped();
const uInt16 = new UInt16();
const vector3 = new Vector3();
const vector3Mag = new Vector3Mag();
const vector3Stamped = new Vector3Stamped();


export function GetRosMessageObject(type: string): any {
  console.log(type, vector3Stamped.type);
  switch (type) {
    case compressedImage.type:
      return new CompressedImage();
    case float32.type:
      return new Float32();
    case float64.type:
      return new Float64();
    case header.type:
      return new Header();
    case image.type:
      return new Image();
    case imu.type:
      return new Imu();
    case log.type:
      return new Log();
    case navSatFix.type:
      return new NavSatFix();
    case navSatStatus.type:
      return new NavSatStatus();
    case pointCloud2.type:
      return new PointCloud2();
    case pointField.type:
      return new PointField();
    case quaternion.type:
      return new Quaternion();
    case quaternionStamped.type:
      return new QuaternionStamped();
    case tFMessage.type:
      return new TFMessage();
    case transform.type:
      return new Transform();
    case transformStamped.type:
      return new TransformStamped();
    case twist.type:
      return new Twist();
    case twistStamped.type:
      return new TwistStamped();
    case uInt16.type:
      return new UInt16();
    case vector3.type:
      return new Vector3();
    case vector3Mag.type:
      return new Vector3Mag();
    case vector3Stamped.type:
      return new Vector3Stamped();
  }

}
