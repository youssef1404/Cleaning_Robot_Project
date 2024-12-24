import PropTypes from 'prop-types';
import { Group, Switch } from '@mantine/core';

export function ControlModes({ isConnected, isAutoMode, isMechanismEnabled, onAutoModeToggle, onMechanismToggle }) {
  return (
    <Group position="apart">
      <Switch
        label="Auto Mode"
        checked={isAutoMode}
        onChange={onAutoModeToggle}
        disabled={!isConnected}
      />
      <Switch
        label="Mechanism"
        checked={isMechanismEnabled}
        onChange={onMechanismToggle}
        disabled={!isConnected}
      />
    </Group>
  );
}

ControlModes.propTypes = {
  isConnected: PropTypes.bool.isRequired,
  isAutoMode: PropTypes.bool.isRequired,
  isMechanismEnabled: PropTypes.bool.isRequired,
  onAutoModeToggle: PropTypes.func.isRequired,
  onMechanismToggle: PropTypes.func.isRequired,
};