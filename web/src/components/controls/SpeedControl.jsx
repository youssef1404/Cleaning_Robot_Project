import React from 'react';
import { Box, Slider, Typography } from '@mui/material';

function SpeedControl({ speed, isAutoMode, onSpeedChange, isMobile }) {
  return (
    <Box 
      sx={{ 
        width: '100%', 
        opacity: isAutoMode ? 0.5 : 1,
        pointerEvents: isAutoMode ? 'none' : 'auto',
        transition: 'opacity 0.3s ease'
      }}
    >
      <Typography gutterBottom color="primary" sx={{ fontWeight: 500 }}>
        Speed Control: {speed}%
      </Typography>
      <Slider
        value={speed}
        onChange={onSpeedChange}
        aria-label="Speed"
        valueLabelDisplay="auto"
        sx={{
          '& .MuiSlider-thumb': {
            width: isMobile ? 20 : 24,
            height: isMobile ? 20 : 24,
          },
          '& .MuiSlider-track': {
            transition: 'background-color 0.3s ease'
          }
        }}
      />
    </Box>
  );
}

export default SpeedControl;