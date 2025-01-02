import { useState, useEffect } from "react";
import { robotApi } from "../services/robotApi";

export function useRobotControl() {
	const [speed, setSpeed] = useState(50);
	const [isAutoMode, setIsAutoMode] = useState(false);
	const [isEnabled, setIsEnabled] = useState(false);
	const [isMagnetEnabled, setIsMagnetEnabled] = useState(false);
	const link = "/api";
	const [lastDirection, setLastDirection] = useState(null);

	const handleJoystickMove = async (event) => {
		const { direction } = event;

		// Only send request if direction changed
		if (direction === lastDirection) return;
		setLastDirection(direction);

		console.log("Joystick moved:", direction);

		try {
			if (!direction) {
				await robotApi.stop(link);
				return;
			}

			switch (direction) {
				case "FORWARD":
					await robotApi.moveForward(link);
					break;
				case "BACKWARD":
					await robotApi.moveBackward(link);
					break;
				case "RIGHT":
					await robotApi.moveRight(link);
					break;
				case "LEFT":
					await robotApi.moveLeft(link);
					break;
			}
		} catch (error) {
			console.error("Control error:", error);
		}
	};

	const handleSpeedChange = (event, newValue) => {
		if (!isAutoMode) setSpeed(newValue);
		robotApi.setSpeed(link, newValue);
	};

	const handleModeChange = (event) => {
		setIsAutoMode(event.target.checked);
	};

	const handleEmergencyStop = async () => {
		setIsEnabled(false);
		setSpeed(0);
		if (link) {
			try {
				await robotApi.stop(link);
			} catch (error) {
				console.error("Emergency stop failed:", error);
			}
		}
	};

	const handleEnableToggle = async () => {
		setIsEnabled(!isEnabled);
		if (!isEnabled) {
			await robotApi.enableMechanism(link);
		} else {
			await robotApi.disableMechanism(link);
		}
	};

	const handleEnableMagnetToggle = async () => {
		setIsMagnetEnabled(!isMagnetEnabled);
		if (!isMagnetEnabled) {
			await robotApi.enableMagnet(link);
		} else {
			await robotApi.disableMagnet(link);
		}
	};

	return {
		speed,
		isAutoMode,
		isEnabled,
		isMagnetEnabled,
		setIsMagnetEnabled,
		handleSpeedChange,
		handleJoystickMove,
		handleModeChange,
		handleEmergencyStop,
		handleEnableToggle,
		handleEnableMagnetToggle,
	};
}
