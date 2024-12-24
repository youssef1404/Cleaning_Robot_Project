import PropTypes from 'prop-types';
import { Box } from '@mantine/core';
import { Joystick } from 'react-joystick-component';
import { useROS } from '../../hooks/useROS';

export function DirectionControl({ disabled }) {
  const { sendVelocity } = useROS();

  const handleMove = (event) => {
    if (disabled) return;
    sendVelocity(event.y, -event.x);
  };

  const handleStop = () => {
    if (disabled) return;
    sendVelocity(0, 0);
  };

  return (
    <Box sx={{ opacity: disabled ? 0.5 : 1, pointerEvents: disabled ? 'none' : 'all' }}>
      <Joystick 
        size={150}
        baseColor="#f0f0f0"
        stickColor="#1971c2"
        move={handleMove}
        stop={handleStop}
      />
    </Box>
  );
}

DirectionControl.propTypes = {
  disabled: PropTypes.bool,
};