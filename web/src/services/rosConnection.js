import ROSLIB from 'roslibjs';
import { useControlStore } from '../store/controlStore';

class ROSConnection {
  constructor() {
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'
    });
    this.setupTopics();
    this.setupListeners();
  }

  setupTopics() {
    this.topics = {
      cmdVel: new ROSLIB.Topic({
        ros: this.ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
      }),
      pid: new ROSLIB.Topic({
        ros: this.ros,
        name: '/pid_params',
        messageType: 'std_msgs/Float32MultiArray'
      }),
      mode: new ROSLIB.Topic({
        ros: this.ros,
        name: '/control_mode',
        messageType: 'std_msgs/String'
      })
    };
  }

  setupListeners() {
    this.ros.on('connection', () => {
      useControlStore.getState().setConnected(true);
      console.log('Connected to ROS bridge');
    });

    this.ros.on('error', (error) => {
      console.error('ROS connection error:', error);
    });

    this.ros.on('close', () => {
      useControlStore.getState().setConnected(false);
      console.log('Connection to ROS bridge closed');
    });
  }

  sendVelocity(linear, angular) {
    const twist = new ROSLIB.Message({
      linear: { x: linear, y: 0, z: 0 },
      angular: { x: 0, y: 0, z: angular }
    });
    this.topics.cmdVel.publish(twist);
  }

  sendPIDParams(p, i, d) {
    const pidMsg = new ROSLIB.Message({
      data: [p, i, d]
    });
    this.topics.pid.publish(pidMsg);
  }

  sendMode(mode) {
    const modeMsg = new ROSLIB.Message({
      data: mode
    });
    this.topics.mode.publish(modeMsg);
  }
}

export const rosConnection = new ROSConnection();