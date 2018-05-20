
"use strict";

let Vibration = require('./Vibration.js');
let CommandCode = require('./CommandCode.js');
let HomePosition = require('./HomePosition.js');
let HilActuatorControls = require('./HilActuatorControls.js');
let FileEntry = require('./FileEntry.js');
let AttitudeTarget = require('./AttitudeTarget.js');
let RadioStatus = require('./RadioStatus.js');
let WaypointReached = require('./WaypointReached.js');
let RCIn = require('./RCIn.js');
let Command = require('./Command.js');
let VFR_HUD = require('./VFR_HUD.js');
let WaypointList = require('./WaypointList.js');
let BatteryStatus = require('./BatteryStatus.js');
let OpticalFlowRad = require('./OpticalFlowRad.js');
let ManualControl = require('./ManualControl.js');
let HilControls = require('./HilControls.js');
let HilStateQuaternion = require('./HilStateQuaternion.js');
let ActuatorControl = require('./ActuatorControl.js');
let State = require('./State.js');
let CamIMUStamp = require('./CamIMUStamp.js');
let PositionTarget = require('./PositionTarget.js');
let OverrideRCIn = require('./OverrideRCIn.js');
let DebugValue = require('./DebugValue.js');
let HilSensor = require('./HilSensor.js');
let ExtendedState = require('./ExtendedState.js');
let StatusText = require('./StatusText.js');
let ParamValue = require('./ParamValue.js');
let ADSBVehicle = require('./ADSBVehicle.js');
let Waypoint = require('./Waypoint.js');
let Thrust = require('./Thrust.js');
let Altitude = require('./Altitude.js');
let Mavlink = require('./Mavlink.js');
let RCOut = require('./RCOut.js');
let GlobalPositionTarget = require('./GlobalPositionTarget.js');
let HilGPS = require('./HilGPS.js');

module.exports = {
  Vibration: Vibration,
  CommandCode: CommandCode,
  HomePosition: HomePosition,
  HilActuatorControls: HilActuatorControls,
  FileEntry: FileEntry,
  AttitudeTarget: AttitudeTarget,
  RadioStatus: RadioStatus,
  WaypointReached: WaypointReached,
  RCIn: RCIn,
  Command: Command,
  VFR_HUD: VFR_HUD,
  WaypointList: WaypointList,
  BatteryStatus: BatteryStatus,
  OpticalFlowRad: OpticalFlowRad,
  ManualControl: ManualControl,
  HilControls: HilControls,
  HilStateQuaternion: HilStateQuaternion,
  ActuatorControl: ActuatorControl,
  State: State,
  CamIMUStamp: CamIMUStamp,
  PositionTarget: PositionTarget,
  OverrideRCIn: OverrideRCIn,
  DebugValue: DebugValue,
  HilSensor: HilSensor,
  ExtendedState: ExtendedState,
  StatusText: StatusText,
  ParamValue: ParamValue,
  ADSBVehicle: ADSBVehicle,
  Waypoint: Waypoint,
  Thrust: Thrust,
  Altitude: Altitude,
  Mavlink: Mavlink,
  RCOut: RCOut,
  GlobalPositionTarget: GlobalPositionTarget,
  HilGPS: HilGPS,
};
