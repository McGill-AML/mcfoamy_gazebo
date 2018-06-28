// Auto-generated. Do not edit!

// (in-package gazebo_example.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class actuator {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.u1 = null;
      this.u2 = null;
      this.u3 = null;
      this.u4 = null;
    }
    else {
      if (initObj.hasOwnProperty('u1')) {
        this.u1 = initObj.u1
      }
      else {
        this.u1 = 0.0;
      }
      if (initObj.hasOwnProperty('u2')) {
        this.u2 = initObj.u2
      }
      else {
        this.u2 = 0.0;
      }
      if (initObj.hasOwnProperty('u3')) {
        this.u3 = initObj.u3
      }
      else {
        this.u3 = 0.0;
      }
      if (initObj.hasOwnProperty('u4')) {
        this.u4 = initObj.u4
      }
      else {
        this.u4 = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type actuator
    // Serialize message field [u1]
    bufferOffset = _serializer.float64(obj.u1, buffer, bufferOffset);
    // Serialize message field [u2]
    bufferOffset = _serializer.float64(obj.u2, buffer, bufferOffset);
    // Serialize message field [u3]
    bufferOffset = _serializer.float64(obj.u3, buffer, bufferOffset);
    // Serialize message field [u4]
    bufferOffset = _serializer.float64(obj.u4, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type actuator
    let len;
    let data = new actuator(null);
    // Deserialize message field [u1]
    data.u1 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [u2]
    data.u2 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [u3]
    data.u3 = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [u4]
    data.u4 = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'gazebo_example/actuator';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '278bbb64bccc0a26b221d16071445863';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 u1
    float64 u2
    float64 u3
    float64 u4
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new actuator(null);
    if (msg.u1 !== undefined) {
      resolved.u1 = msg.u1;
    }
    else {
      resolved.u1 = 0.0
    }

    if (msg.u2 !== undefined) {
      resolved.u2 = msg.u2;
    }
    else {
      resolved.u2 = 0.0
    }

    if (msg.u3 !== undefined) {
      resolved.u3 = msg.u3;
    }
    else {
      resolved.u3 = 0.0
    }

    if (msg.u4 !== undefined) {
      resolved.u4 = msg.u4;
    }
    else {
      resolved.u4 = 0.0
    }

    return resolved;
    }
};

module.exports = actuator;
