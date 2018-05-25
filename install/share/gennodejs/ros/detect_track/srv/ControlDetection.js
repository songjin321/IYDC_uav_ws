// Auto-generated. Do not edit!

// (in-package detect_track.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class ControlDetectionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ControlType = null;
    }
    else {
      if (initObj.hasOwnProperty('ControlType')) {
        this.ControlType = initObj.ControlType
      }
      else {
        this.ControlType = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlDetectionRequest
    // Serialize message field [ControlType]
    bufferOffset = _serializer.int8(obj.ControlType, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlDetectionRequest
    let len;
    let data = new ControlDetectionRequest(null);
    // Deserialize message field [ControlType]
    data.ControlType = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'detect_track/ControlDetectionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '506f1fd1bdb2c9c02a91632248441f14';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 ControlType
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlDetectionRequest(null);
    if (msg.ControlType !== undefined) {
      resolved.ControlType = msg.ControlType;
    }
    else {
      resolved.ControlType = 0
    }

    return resolved;
    }
};

class ControlDetectionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.isOk = null;
    }
    else {
      if (initObj.hasOwnProperty('isOk')) {
        this.isOk = initObj.isOk
      }
      else {
        this.isOk = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ControlDetectionResponse
    // Serialize message field [isOk]
    bufferOffset = _serializer.bool(obj.isOk, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ControlDetectionResponse
    let len;
    let data = new ControlDetectionResponse(null);
    // Deserialize message field [isOk]
    data.isOk = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'detect_track/ControlDetectionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0ba8951a653c7742f358b4db6e37fb3b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool isOk
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ControlDetectionResponse(null);
    if (msg.isOk !== undefined) {
      resolved.isOk = msg.isOk;
    }
    else {
      resolved.isOk = false
    }

    return resolved;
    }
};

module.exports = {
  Request: ControlDetectionRequest,
  Response: ControlDetectionResponse,
  md5sum() { return 'b418a6e2e6ff2264b54f7352bb0ed12b'; },
  datatype() { return 'detect_track/ControlDetection'; }
};
