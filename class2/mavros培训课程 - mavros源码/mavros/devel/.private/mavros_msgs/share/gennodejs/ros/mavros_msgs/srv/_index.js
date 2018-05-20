
"use strict";

let FileRename = require('./FileRename.js')
let WaypointClear = require('./WaypointClear.js')
let FileRemove = require('./FileRemove.js')
let ParamPull = require('./ParamPull.js')
let FileList = require('./FileList.js')
let CommandLong = require('./CommandLong.js')
let CommandBool = require('./CommandBool.js')
let StreamRate = require('./StreamRate.js')
let ParamGet = require('./ParamGet.js')
let FileChecksum = require('./FileChecksum.js')
let CommandTOL = require('./CommandTOL.js')
let ParamSet = require('./ParamSet.js')
let CommandHome = require('./CommandHome.js')
let FileRemoveDir = require('./FileRemoveDir.js')
let FileRead = require('./FileRead.js')
let WaypointPull = require('./WaypointPull.js')
let WaypointPush = require('./WaypointPush.js')
let SetMavFrame = require('./SetMavFrame.js')
let CommandInt = require('./CommandInt.js')
let FileTruncate = require('./FileTruncate.js')
let SetMode = require('./SetMode.js')
let FileWrite = require('./FileWrite.js')
let ParamPush = require('./ParamPush.js')
let FileClose = require('./FileClose.js')
let WaypointSetCurrent = require('./WaypointSetCurrent.js')
let FileOpen = require('./FileOpen.js')
let CommandTriggerControl = require('./CommandTriggerControl.js')
let FileMakeDir = require('./FileMakeDir.js')

module.exports = {
  FileRename: FileRename,
  WaypointClear: WaypointClear,
  FileRemove: FileRemove,
  ParamPull: ParamPull,
  FileList: FileList,
  CommandLong: CommandLong,
  CommandBool: CommandBool,
  StreamRate: StreamRate,
  ParamGet: ParamGet,
  FileChecksum: FileChecksum,
  CommandTOL: CommandTOL,
  ParamSet: ParamSet,
  CommandHome: CommandHome,
  FileRemoveDir: FileRemoveDir,
  FileRead: FileRead,
  WaypointPull: WaypointPull,
  WaypointPush: WaypointPush,
  SetMavFrame: SetMavFrame,
  CommandInt: CommandInt,
  FileTruncate: FileTruncate,
  SetMode: SetMode,
  FileWrite: FileWrite,
  ParamPush: ParamPush,
  FileClose: FileClose,
  WaypointSetCurrent: WaypointSetCurrent,
  FileOpen: FileOpen,
  CommandTriggerControl: CommandTriggerControl,
  FileMakeDir: FileMakeDir,
};
