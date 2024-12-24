import { MantineProvider } from "@mantine/core";
import { useControlStore } from "./store/controlStore";
import { ControlModes } from "./components/controls/ControlModes";
import { PIDControl } from "./components/pid/PIDControl";
import { DirectionControl } from "./components/movement/DirectionControl";
import { EmergencyStop } from "./components/controls/EmergencyStop";

export default function App() {
	const {
		isConnected,
		isAutoMode,
		isMechanismEnabled,
		pid,
		toggleAutoMode,
		toggleMechanism,
		updatePID,
	} = useControlStore();

	return (
		<MantineProvider>
			<div
				style={{
					maxWidth: 600,
					margin: "2rem auto",
					padding: "0 1rem",
				}}
			>
				<h1>ESP32 Control Panel</h1>

				<ControlModes
					isConnected={isConnected}
					isAutoMode={isAutoMode}
					isMechanismEnabled={isMechanismEnabled}
					onAutoModeToggle={toggleAutoMode}
					onMechanismToggle={toggleMechanism}
				/>

				<PIDControl
					pid={pid}
					onChange={updatePID}
					disabled={!isConnected}
				/>

				<DirectionControl disabled={!isConnected || isAutoMode} />

				<EmergencyStop disabled={!isConnected} />
			</div>
		</MantineProvider>
	);
}
