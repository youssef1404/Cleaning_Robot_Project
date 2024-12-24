import PropTypes from 'prop-types';
import { NumberInput, Group, Stack, Text } from '@mantine/core';

export function PIDControl({ pid, onChange, disabled }) {
  return (
    <Stack spacing="xs">
      <Text size="lg" weight={500}>PID Controls</Text>
      <Group grow>
        <NumberInput
          label="P"
          value={pid.p}
          onChange={(val) => onChange({ ...pid, p: Number(val) })}
          precision={2}
          step={0.1}
          disabled={disabled}
        />
        <NumberInput
          label="I"
          value={pid.i}
          onChange={(val) => onChange({ ...pid, i: Number(val) })}
          precision={2}
          step={0.1}
          disabled={disabled}
        />
        <NumberInput
          label="D"
          value={pid.d}
          onChange={(val) => onChange({ ...pid, d: Number(val) })}
          precision={2}
          step={0.1}
          disabled={disabled}
        />
      </Group>
    </Stack>
  );
}

PIDControl.propTypes = {
  pid: PropTypes.shape({
    p: PropTypes.number.isRequired,
    i: PropTypes.number.isRequired,
    d: PropTypes.number.isRequired,
  }).isRequired,
  onChange: PropTypes.func.isRequired,
  disabled: PropTypes.bool,
};