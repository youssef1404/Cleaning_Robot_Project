import React, { useState } from "react";
import { Box, Paper, useMediaQuery, useTheme } from "@mui/material";
import ControlHeader from "./controls/ControlHeader";
import ModeToggle from "./controls/ModeToggle";
import SpeedControl from "./controls/SpeedControl";
import JoystickControl from "./controls/JoystickControl";
import ControlButtons from "./controls/ControlButtons";
import { useRobotControl } from "../hooks/useRobotControl";

function RobotControl() {
	const theme = useTheme();
	const isMobile = useMediaQuery(theme.breakpoints.down("sm"));
	const {
		speed,
		isAutoMode,
		isEnabled,
		isMagnetEnabled,
		handleSpeedChange,
		handleJoystickMove,
		handleModeChange,
		handleEmergencyStop,
		handleEnableToggle,
		handleEnableMagnetToggle,
	} = useRobotControl();

	return (
		<Paper
			elevation={3}
			sx={{
				maxWidth: isMobile ? "100%" : 500,
				width: "100%",
				margin: "auto",
				padding: isMobile ? 2 : 4,
				display: "flex",
				flexDirection: "column",
				gap: 3,
				bgcolor: "rgba(255, 255, 255, 0.95)",
				borderRadius: 3,
				backdropFilter: "blur(10px)",
			}}
		>
			<ControlHeader isMobile={isMobile} />

			<Box sx={{ display: "flex", flexDirection: "column", gap: 3 }}>
				<ModeToggle
					isAutoMode={isAutoMode}
					onModeChange={handleModeChange}
				/>

				<SpeedControl
					speed={speed}
					isAutoMode={isAutoMode}
					onSpeedChange={handleSpeedChange}
					isMobile={isMobile}
				/>

				<JoystickControl
					isAutoMode={isAutoMode}
					onMove={handleJoystickMove}
					isMobile={isMobile}
				/>

				<ControlButtons
					isAutoMode={isAutoMode}
					isEnabled={isEnabled}
					isMagnetEnabled={isMagnetEnabled}
					onEmergencyStop={handleEmergencyStop}
					onEnableToggle={handleEnableToggle}
					onEnableMagnetToggle={handleEnableMagnetToggle}
					isMobile={isMobile}
				/>
			</Box>
		</Paper>
	);
}

export default RobotControl;
