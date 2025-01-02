import React from 'react';
import { Box } from '@mui/material';
import { Joystick } from 'react-joystick-component';

function JoystickControl({ isAutoMode, onMove, isMobile }) {
  return (
    <Box sx={{ 
      bgcolor: '#f5f5f5',
      padding: 3,
      borderRadius: 3,
      display: 'flex',
      justifyContent: 'center',
      opacity: isAutoMode ? 0.5 : 1,
      pointerEvents: isAutoMode ? 'none' : 'auto',
      transition: 'opacity 0.3s ease',
      boxShadow: 'inset 0 2px 4px rgba(0,0,0,0.1)'
    }}>
      <Joystick 
        size={isMobile ? 120 : 180}
        baseColor="#e0e0e0"
        stickColor="#1976d2"
        move={onMove}
      />
    </Box>
  );
}

export default JoystickControl;