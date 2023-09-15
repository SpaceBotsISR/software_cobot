// Auto-generated. Do not edit!

// (in-package space_cobot_mpc_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class pwm_values {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.Force = null;
      this.Moment = null;
      this.Actuation = null;
      this.PWM = null;
      this.rpm = null;
      this.Att_P_gains = null;
      this.Att_I_gains = null;
      this.Att_D_gains = null;
      this.Current_rpy = null;
      this.Desired_rpy = null;
      this.Desired_position = null;
      this.Desired_velocity = null;
      this.Desired_omega = null;
      this.Error_rpy = null;
      this.err_int = null;
      this.Time_Step = null;
      this.Quat_error = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('Force')) {
        this.Force = initObj.Force
      }
      else {
        this.Force = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Moment')) {
        this.Moment = initObj.Moment
      }
      else {
        this.Moment = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Actuation')) {
        this.Actuation = initObj.Actuation
      }
      else {
        this.Actuation = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('PWM')) {
        this.PWM = initObj.PWM
      }
      else {
        this.PWM = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('rpm')) {
        this.rpm = initObj.rpm
      }
      else {
        this.rpm = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('Att_P_gains')) {
        this.Att_P_gains = initObj.Att_P_gains
      }
      else {
        this.Att_P_gains = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Att_I_gains')) {
        this.Att_I_gains = initObj.Att_I_gains
      }
      else {
        this.Att_I_gains = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Att_D_gains')) {
        this.Att_D_gains = initObj.Att_D_gains
      }
      else {
        this.Att_D_gains = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Current_rpy')) {
        this.Current_rpy = initObj.Current_rpy
      }
      else {
        this.Current_rpy = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Desired_rpy')) {
        this.Desired_rpy = initObj.Desired_rpy
      }
      else {
        this.Desired_rpy = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Desired_position')) {
        this.Desired_position = initObj.Desired_position
      }
      else {
        this.Desired_position = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Desired_velocity')) {
        this.Desired_velocity = initObj.Desired_velocity
      }
      else {
        this.Desired_velocity = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Desired_omega')) {
        this.Desired_omega = initObj.Desired_omega
      }
      else {
        this.Desired_omega = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Error_rpy')) {
        this.Error_rpy = initObj.Error_rpy
      }
      else {
        this.Error_rpy = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('err_int')) {
        this.err_int = initObj.err_int
      }
      else {
        this.err_int = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('Time_Step')) {
        this.Time_Step = initObj.Time_Step
      }
      else {
        this.Time_Step = 0.0;
      }
      if (initObj.hasOwnProperty('Quat_error')) {
        this.Quat_error = initObj.Quat_error
      }
      else {
        this.Quat_error = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type pwm_values
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [Force] has the right length
    if (obj.Force.length !== 3) {
      throw new Error('Unable to serialize array field Force - length must be 3')
    }
    // Serialize message field [Force]
    bufferOffset = _arraySerializer.float64(obj.Force, buffer, bufferOffset, 3);
    // Check that the constant length array field [Moment] has the right length
    if (obj.Moment.length !== 3) {
      throw new Error('Unable to serialize array field Moment - length must be 3')
    }
    // Serialize message field [Moment]
    bufferOffset = _arraySerializer.float64(obj.Moment, buffer, bufferOffset, 3);
    // Check that the constant length array field [Actuation] has the right length
    if (obj.Actuation.length !== 6) {
      throw new Error('Unable to serialize array field Actuation - length must be 6')
    }
    // Serialize message field [Actuation]
    bufferOffset = _arraySerializer.float64(obj.Actuation, buffer, bufferOffset, 6);
    // Check that the constant length array field [PWM] has the right length
    if (obj.PWM.length !== 6) {
      throw new Error('Unable to serialize array field PWM - length must be 6')
    }
    // Serialize message field [PWM]
    bufferOffset = _arraySerializer.float64(obj.PWM, buffer, bufferOffset, 6);
    // Check that the constant length array field [rpm] has the right length
    if (obj.rpm.length !== 6) {
      throw new Error('Unable to serialize array field rpm - length must be 6')
    }
    // Serialize message field [rpm]
    bufferOffset = _arraySerializer.float64(obj.rpm, buffer, bufferOffset, 6);
    // Check that the constant length array field [Att_P_gains] has the right length
    if (obj.Att_P_gains.length !== 3) {
      throw new Error('Unable to serialize array field Att_P_gains - length must be 3')
    }
    // Serialize message field [Att_P_gains]
    bufferOffset = _arraySerializer.float64(obj.Att_P_gains, buffer, bufferOffset, 3);
    // Check that the constant length array field [Att_I_gains] has the right length
    if (obj.Att_I_gains.length !== 3) {
      throw new Error('Unable to serialize array field Att_I_gains - length must be 3')
    }
    // Serialize message field [Att_I_gains]
    bufferOffset = _arraySerializer.float64(obj.Att_I_gains, buffer, bufferOffset, 3);
    // Check that the constant length array field [Att_D_gains] has the right length
    if (obj.Att_D_gains.length !== 3) {
      throw new Error('Unable to serialize array field Att_D_gains - length must be 3')
    }
    // Serialize message field [Att_D_gains]
    bufferOffset = _arraySerializer.float64(obj.Att_D_gains, buffer, bufferOffset, 3);
    // Check that the constant length array field [Current_rpy] has the right length
    if (obj.Current_rpy.length !== 3) {
      throw new Error('Unable to serialize array field Current_rpy - length must be 3')
    }
    // Serialize message field [Current_rpy]
    bufferOffset = _arraySerializer.float64(obj.Current_rpy, buffer, bufferOffset, 3);
    // Check that the constant length array field [Desired_rpy] has the right length
    if (obj.Desired_rpy.length !== 3) {
      throw new Error('Unable to serialize array field Desired_rpy - length must be 3')
    }
    // Serialize message field [Desired_rpy]
    bufferOffset = _arraySerializer.float64(obj.Desired_rpy, buffer, bufferOffset, 3);
    // Check that the constant length array field [Desired_position] has the right length
    if (obj.Desired_position.length !== 3) {
      throw new Error('Unable to serialize array field Desired_position - length must be 3')
    }
    // Serialize message field [Desired_position]
    bufferOffset = _arraySerializer.float64(obj.Desired_position, buffer, bufferOffset, 3);
    // Check that the constant length array field [Desired_velocity] has the right length
    if (obj.Desired_velocity.length !== 3) {
      throw new Error('Unable to serialize array field Desired_velocity - length must be 3')
    }
    // Serialize message field [Desired_velocity]
    bufferOffset = _arraySerializer.float64(obj.Desired_velocity, buffer, bufferOffset, 3);
    // Check that the constant length array field [Desired_omega] has the right length
    if (obj.Desired_omega.length !== 3) {
      throw new Error('Unable to serialize array field Desired_omega - length must be 3')
    }
    // Serialize message field [Desired_omega]
    bufferOffset = _arraySerializer.float64(obj.Desired_omega, buffer, bufferOffset, 3);
    // Check that the constant length array field [Error_rpy] has the right length
    if (obj.Error_rpy.length !== 3) {
      throw new Error('Unable to serialize array field Error_rpy - length must be 3')
    }
    // Serialize message field [Error_rpy]
    bufferOffset = _arraySerializer.float64(obj.Error_rpy, buffer, bufferOffset, 3);
    // Check that the constant length array field [err_int] has the right length
    if (obj.err_int.length !== 3) {
      throw new Error('Unable to serialize array field err_int - length must be 3')
    }
    // Serialize message field [err_int]
    bufferOffset = _arraySerializer.float64(obj.err_int, buffer, bufferOffset, 3);
    // Serialize message field [Time_Step]
    bufferOffset = _serializer.float64(obj.Time_Step, buffer, bufferOffset);
    // Check that the constant length array field [Quat_error] has the right length
    if (obj.Quat_error.length !== 3) {
      throw new Error('Unable to serialize array field Quat_error - length must be 3')
    }
    // Serialize message field [Quat_error]
    bufferOffset = _arraySerializer.float64(obj.Quat_error, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type pwm_values
    let len;
    let data = new pwm_values(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [Force]
    data.Force = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Moment]
    data.Moment = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Actuation]
    data.Actuation = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [PWM]
    data.PWM = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [rpm]
    data.rpm = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [Att_P_gains]
    data.Att_P_gains = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Att_I_gains]
    data.Att_I_gains = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Att_D_gains]
    data.Att_D_gains = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Current_rpy]
    data.Current_rpy = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Desired_rpy]
    data.Desired_rpy = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Desired_position]
    data.Desired_position = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Desired_velocity]
    data.Desired_velocity = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Desired_omega]
    data.Desired_omega = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Error_rpy]
    data.Error_rpy = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [err_int]
    data.err_int = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    // Deserialize message field [Time_Step]
    data.Time_Step = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [Quat_error]
    data.Quat_error = _arrayDeserializer.float64(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 464;
  }

  static datatype() {
    // Returns string type for a message object
    return 'space_cobot_mpc_controller/pwm_values';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'da432c94c450f85a4b2d4b18064bf11c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    float64[3] Force
    float64[3] Moment
    float64[6] Actuation
    float64[6] PWM
    float64[6] rpm
    float64[3] Att_P_gains
    float64[3] Att_I_gains
    float64[3] Att_D_gains
    float64[3] Current_rpy
    float64[3] Desired_rpy
    float64[3] Desired_position
    float64[3] Desired_velocity
    float64[3] Desired_omega
    float64[3] Error_rpy
    float64[3] err_int
    float64 Time_Step
    float64[3] Quat_error
    
    
    
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
    const resolved = new pwm_values(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.Force !== undefined) {
      resolved.Force = msg.Force;
    }
    else {
      resolved.Force = new Array(3).fill(0)
    }

    if (msg.Moment !== undefined) {
      resolved.Moment = msg.Moment;
    }
    else {
      resolved.Moment = new Array(3).fill(0)
    }

    if (msg.Actuation !== undefined) {
      resolved.Actuation = msg.Actuation;
    }
    else {
      resolved.Actuation = new Array(6).fill(0)
    }

    if (msg.PWM !== undefined) {
      resolved.PWM = msg.PWM;
    }
    else {
      resolved.PWM = new Array(6).fill(0)
    }

    if (msg.rpm !== undefined) {
      resolved.rpm = msg.rpm;
    }
    else {
      resolved.rpm = new Array(6).fill(0)
    }

    if (msg.Att_P_gains !== undefined) {
      resolved.Att_P_gains = msg.Att_P_gains;
    }
    else {
      resolved.Att_P_gains = new Array(3).fill(0)
    }

    if (msg.Att_I_gains !== undefined) {
      resolved.Att_I_gains = msg.Att_I_gains;
    }
    else {
      resolved.Att_I_gains = new Array(3).fill(0)
    }

    if (msg.Att_D_gains !== undefined) {
      resolved.Att_D_gains = msg.Att_D_gains;
    }
    else {
      resolved.Att_D_gains = new Array(3).fill(0)
    }

    if (msg.Current_rpy !== undefined) {
      resolved.Current_rpy = msg.Current_rpy;
    }
    else {
      resolved.Current_rpy = new Array(3).fill(0)
    }

    if (msg.Desired_rpy !== undefined) {
      resolved.Desired_rpy = msg.Desired_rpy;
    }
    else {
      resolved.Desired_rpy = new Array(3).fill(0)
    }

    if (msg.Desired_position !== undefined) {
      resolved.Desired_position = msg.Desired_position;
    }
    else {
      resolved.Desired_position = new Array(3).fill(0)
    }

    if (msg.Desired_velocity !== undefined) {
      resolved.Desired_velocity = msg.Desired_velocity;
    }
    else {
      resolved.Desired_velocity = new Array(3).fill(0)
    }

    if (msg.Desired_omega !== undefined) {
      resolved.Desired_omega = msg.Desired_omega;
    }
    else {
      resolved.Desired_omega = new Array(3).fill(0)
    }

    if (msg.Error_rpy !== undefined) {
      resolved.Error_rpy = msg.Error_rpy;
    }
    else {
      resolved.Error_rpy = new Array(3).fill(0)
    }

    if (msg.err_int !== undefined) {
      resolved.err_int = msg.err_int;
    }
    else {
      resolved.err_int = new Array(3).fill(0)
    }

    if (msg.Time_Step !== undefined) {
      resolved.Time_Step = msg.Time_Step;
    }
    else {
      resolved.Time_Step = 0.0
    }

    if (msg.Quat_error !== undefined) {
      resolved.Quat_error = msg.Quat_error;
    }
    else {
      resolved.Quat_error = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = pwm_values;
