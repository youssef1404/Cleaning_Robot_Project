import PropTypes from 'prop-types';
import { Button } from '@mantine/core';
import { useROS } from '../../hooks/useROS';

export function EmergencyStop({ disabled }) {
  const { sendEmergencyStop } = useROS();

  return (
    <Button 
      color="red" 
      size="lg"
      disabled={disabled}
      onClick={sendEmergencyStop}
    >
      EMERGENCY STOP
    </Button>
  );
}

EmergencyStop.propTypes = {
  disabled: PropTypes.bool,
};