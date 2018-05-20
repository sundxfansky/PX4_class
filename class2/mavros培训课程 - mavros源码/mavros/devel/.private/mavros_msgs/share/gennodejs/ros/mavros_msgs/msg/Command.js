// Auto-generated. Do not edit!

// (in-package mavros_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Command {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.comid = null;
      this.command = null;
      this.sub_mode = null;
      this.pos_sp = null;
      this.vel_sp = null;
      this.yaw_sp = null;
      this.yaw_rate_sp = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('comid')) {
        this.comid = initObj.comid
      }
      else {
        this.comid = 0;
      }
      if (initObj.hasOwnProperty('command')) {
        this.command = initObj.command
      }
      else {
        this.command = 0;
      }
      if (initObj.hasOwnProperty('sub_mode')) {
        this.sub_mode = initObj.sub_mode
      }
      else {
        this.sub_mode = 0;
      }
      if (initObj.hasOwnProperty('pos_sp')) {
        this.pos_sp = initObj.pos_sp
      }
      else {
        this.pos_sp = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('vel_sp')) {
        this.vel_sp = initObj.vel_sp
      }
      else {
        this.vel_sp = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('yaw_sp')) {
        this.yaw_sp = initObj.yaw_sp
      }
      else {
        this.yaw_sp = 0.0;
      }
      if (initObj.hasOwnProperty('yaw_rate_sp')) {
        this.yaw_rate_sp = initObj.yaw_rate_sp
      }
      else {
        this.yaw_rate_sp = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Command
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [comid]
    bufferOffset = _serializer.uint32(obj.comid, buffer, bufferOffset);
    // Serialize message field [command]
    bufferOffset = _serializer.uint8(obj.command, buffer, bufferOffset);
    // Serialize message field [sub_mode]
    bufferOffset = _serializer.uint8(obj.sub_mode, buffer, bufferOffset);
    // Check that the constant length array field [pos_sp] has the right length
    if (obj.pos_sp.length !== 3) {
      throw new Error('Unable to serialize array field pos_sp - length must be 3')
    }
    // Serialize message field [pos_sp]
    bufferOffset = _arraySerializer.float32(obj.pos_sp, buffer, bufferOffset, 3);
    // Check that the constant length array field [vel_sp] has the right length
    if (obj.vel_sp.length !== 3) {
      throw new Error('Unable to serialize array field vel_sp - length must be 3')
    }
    // Serialize message field [vel_sp]
    bufferOffset = _arraySerializer.float32(obj.vel_sp, buffer, bufferOffset, 3);
    // Serialize message field [yaw_sp]
    bufferOffset = _serializer.float32(obj.yaw_sp, buffer, bufferOffset);
    // Serialize message field [yaw_rate_sp]
    bufferOffset = _serializer.float32(obj.yaw_rate_sp, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Command
    let len;
    let data = new Command(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [comid]
    data.comid = _deserializer.uint32(buffer, bufferOffset);
    // Deserialize message field [command]
    data.command = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [sub_mode]
    data.sub_mode = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [pos_sp]
    data.pos_sp = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [vel_sp]
    data.vel_sp = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    // Deserialize message field [yaw_sp]
    data.yaw_sp = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [yaw_rate_sp]
    data.yaw_rate_sp = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 38;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mavros_msgs/Command';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc0a85df8be3034122897f9d6b86110f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    std_msgs/Header header
    
    #enum Command
    #{
    #    Standby = 0,
    #    Takeoff,
    #    Hold,
    #    Land,
    #    Move,
    #    Disarm,
    #    Moving_Body
    #};
    # sub_mode 2-bit value: 
    # 0 for position, 1 for vel, 1st for xy, 2nd for z.
    #                   xy position     xy velocity
    # z position       	0b00(0)       0b10(2)
    # z velocity		0b01(1)       0b11(3)
    #
    
    uint32 comid
    uint8 command
    uint8 sub_mode
    float32[3] pos_sp
    float32[3] vel_sp
    float32 yaw_sp
    float32 yaw_rate_sp
    
    
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Command(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.comid !== undefined) {
      resolved.comid = msg.comid;
    }
    else {
      resolved.comid = 0
    }

    if (msg.command !== undefined) {
      resolved.command = msg.command;
    }
    else {
      resolved.command = 0
    }

    if (msg.sub_mode !== undefined) {
      resolved.sub_mode = msg.sub_mode;
    }
    else {
      resolved.sub_mode = 0
    }

    if (msg.pos_sp !== undefined) {
      resolved.pos_sp = msg.pos_sp;
    }
    else {
      resolved.pos_sp = new Array(3).fill(0)
    }

    if (msg.vel_sp !== undefined) {
      resolved.vel_sp = msg.vel_sp;
    }
    else {
      resolved.vel_sp = new Array(3).fill(0)
    }

    if (msg.yaw_sp !== undefined) {
      resolved.yaw_sp = msg.yaw_sp;
    }
    else {
      resolved.yaw_sp = 0.0
    }

    if (msg.yaw_rate_sp !== undefined) {
      resolved.yaw_rate_sp = msg.yaw_rate_sp;
    }
    else {
      resolved.yaw_rate_sp = 0.0
    }

    return resolved;
    }
};

module.exports = Command;
