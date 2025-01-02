import React from 'react';
import { Typography } from '@mui/material';

function ControlHeader({ isMobile }) {
  return (
    <Typography 
      variant={isMobile ? "h5" : "h4"} 
      component="h1" 
      align="center"
      color="primary"
      sx={{ 
        fontWeight: 'bold',
        textShadow: '0px 1px 2px rgba(0,0,0,0.1)'
      }}
    >
      Robot Control Panel
    </Typography>
  );
}

export default ControlHeader;