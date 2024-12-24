import { useCallback } from 'react';
import { rosConnection } from '../services/rosConnection';
import { useControlStore } from '../store/controlStore';

export function useROS() {
  const updatePID = useControlStore((state) => state.updatePID);

  const sendVelocity = useCallback((linear, angular) => {
    rosConnection.sendVelocity(linear, angular);
  }, []);

  const sendPIDParams = useCallback((p, i, d) => {
    rosConnection.sendPIDParams(p, i, d);
    updatePID({ p, i, d });
  }, [updatePID]);

  const sendEmergencyStop = useCallback(() => {
    rosConnection.sendVelocity(0, 0);
    rosConnection.sendMode('manual');
  }, []);

  return {
    sendVelocity,
    sendPIDParams,
    sendEmergencyStop,
  };
}