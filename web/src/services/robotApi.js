import axios from "axios";
import { API_CONFIG } from "./apiConfig";

const api = axios.create(API_CONFIG);

export const robotApi = {
	async moveForward(link) {
		try {
			console.log("Moving forward:", link);
			const response = await api.get(`${link}/move-forward`, {
				withCredentials: false,
			});
			console.log("Forward response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Forward error:", error.response || error);
			throw error;
		}
	},

	async moveBackward(link) {
		try {
			console.log("Moving backward:", link);
			const response = await api.get(`${link}/move-backward`, {
				withCredentials: false,
			});
			console.log("Backward response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Backward error:", error.response || error);
			throw error;
		}
	},

	async moveLeft(link) {
		try {
			console.log("Moving left:", link);
			const response = await api.get(`${link}/move-left`, {
				withCredentials: false,
			});
			console.log("Left response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Left error:", error.response || error);
			throw error;
		}
	},

	async moveRight(link) {
		try {
			console.log("Moving right:", link);
			const response = await api.get(`${link}/move-right`, {
				withCredentials: false,
			});
			console.log("Right response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Right error:", error.response || error);
			throw error;
		}
	},

	async stop(link) {
		try {
			console.log("Stopping:", link);
			const response = await api.get(`${link}/stop`, {
				withCredentials: false,
			});
			console.log("Stop response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Stop error:", error.response || error);
			throw error;
		}
	},

	async enableMechanism(link) {
		try {
			console.log("Enabling mechanism:", link);
			const response = await api.get(`${link}/up-servo`, {
				withCredentials: false,
			});
			console.log("Enable response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Enable error:", error.response || error);
			throw error;
		}
	},

	async disableMechanism(link) {
		try {
			console.log("Disabling mechanism:", link);
			const response = await api.get(`${link}/down-servo`, {
				withCredentials: false,
			});
			console.log("Disable response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Disable error:", error.response || error);
			throw error;
		}
	},

	async enableMagnet(link) {
		try {
			console.log("Enabling magnet:", link);
			const response = await api.get(`${link}/enable-magnet`, {
				withCredentials: false,
			});
			console.log("Enable magnet response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Enable magnet error:", error.response || error);
			throw error;
		}
	},

	async disableMagnet(link) {
		try {
			console.log("Disabling magnet:", link);
			const response = await api.get(`${link}/disable-magnet`, {
				withCredentials: false,
			});
			console.log("Disable magnet response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Disable magnet error:", error.response || error);
			throw error;
		}
	},

	async setSpeed(link, speed) {
		try {
			console.log("Setting speed:", link, speed);
			const response = await api.post(
				`${link}/change-speed?speed=${speed}`,
				{
					withCredentials: false,
				}
			);
			console.log("Set speed response:", response.data);
			return response.data;
		} catch (error) {
			console.error("Set speed error:", error.response || error);
			throw error;
		}
	},
};
