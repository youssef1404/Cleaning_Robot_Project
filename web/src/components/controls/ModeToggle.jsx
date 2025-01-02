import React from 'react';
import { FormControlLabel, Switch, Typography } from '@mui/material';

function ModeToggle({ isAutoMode, onModeChange }) {
  return (
    <FormControlLabel
      control={
        <Switch
          checked={isAutoMode}
          onChange={onModeChange}
          color="primary"
        />
      }
      label={
        <Typography color="primary" sx={{ fontWeight: 500 }}>
          {isAutoMode ? 'Automated Mode' : 'Manual Mode'}
        </Typography>
      }
      sx={{ 
        alignSelf: 'center',
        border: '1px solid #e0e0e0',
        borderRadius: 2,
        padding: 1,
        bgcolor: isAutoMode ? 'rgba(25, 118, 210, 0.08)' : 'transparent',
        transition: 'background-color 0.3s ease'
      }}
    />
  );
}

export default ModeToggle;