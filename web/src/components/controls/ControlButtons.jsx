import React from "react";
import { Box, Button } from "@mui/material";

function ControlButtons({
	isAutoMode,
	isEnabled,
	isMagnetEnabled,
	onEmergencyStop,
	onEnableMagnetToggle,
	onEnableToggle,
	isMobile,
}) {
	return (
		<Box
			sx={{
				display: "flex",
				flexDirection: "column",
				gap: 2,
			}}
		>
			<Box
				sx={{
					display: "flex",
					justifyContent: "space-between",
					gap: 5,
				}}
			>
				<Button
					variant="contained"
					color={isEnabled ? "error" : "success"}
					onClick={onEnableToggle}
					disabled={isAutoMode}
					sx={{
						height: isMobile ? 48 : 56,
						fontSize: isMobile ? "1rem" : "1.1rem",
						fontWeight: "bold",
						opacity: isAutoMode ? 0.5 : 1,
						transition: "all 0.3s ease",
						flex: 1,
					}}
				>
					{isEnabled ? "DISABLE MECHANISM" : "ENABLE MECHANISM"}
				</Button>

				<Button
					variant="contained"
					color={isMagnetEnabled ? "error" : "success"}
					onClick={onEnableMagnetToggle}
					disabled={isAutoMode}
					sx={{
						height: isMobile ? 48 : 56,
						fontSize: isMobile ? "1rem" : "1.1rem",
						fontWeight: "bold",
						opacity: isAutoMode ? 0.5 : 1,
						transition: "all 0.3s ease",
						flex: 1,
					}}
				>
					{isMagnetEnabled ? "DISABLE MAGNET" : "ENABLE MAGNET"}
				</Button>
			</Box>

			<Button
				variant="contained"
				color="error"
				onClick={onEmergencyStop}
				sx={{
					height: isMobile ? 56 : 64,
					fontSize: isMobile ? "1.1rem" : "1.25rem",
					fontWeight: "bold",
					background:
						"linear-gradient(45deg, #f44336 30%, #d32f2f 90%)",
					boxShadow: "0 3px 5px 2px rgba(255, 105, 135, .3)",
					"&:hover": {
						background:
							"linear-gradient(45deg, #d32f2f 30%, #b71c1c 90%)",
					},
				}}
			>
				EMERGENCY STOP
			</Button>
		</Box>
	);
}

export default ControlButtons;
