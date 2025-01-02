import { useState } from 'react';

export function useJoystickState() {
  const [lastDirection, setLastDirection] = useState(null);

  const shouldUpdateDirection = (newDirection) => {
    console.log('Joystick state:', {
      newDirection: newDirection || 'NONE',
      lastDirection: lastDirection || 'NONE',
      changed: newDirection !== lastDirection
    });

    if (newDirection !== lastDirection) {
      setLastDirection(newDirection);
      return true;
    }
    return false;
  };

  return { lastDirection, shouldUpdateDirection };
}