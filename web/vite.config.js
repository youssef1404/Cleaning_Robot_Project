import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

// https://vitejs.dev/config/
export default defineConfig({
	plugins: [react()],
	server: {
		proxy: {
			"/api": {
				target: "http://192.168.107.72",
				changeOrigin: true,
				rewrite: (path) => path.replace(/^\/api/, ""), // Remove '/api' prefix
				secure: false,
			},
		},
	},
});
