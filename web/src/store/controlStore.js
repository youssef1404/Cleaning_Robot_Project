import { create } from 'zustand';

export const useControlStore = create((set) => ({
  isConnected: false,
  isAutoMode: false,
  isMechanismEnabled: false,
  pid: { p: 0, i: 0, d: 0 },
  setConnected: (connected) => set({ isConnected: connected }),
  toggleAutoMode: () => set((state) => ({ isAutoMode: !state.isAutoMode })),
  toggleMechanism: () => set((state) => ({ isMechanismEnabled: !state.isMechanismEnabled })),
  updatePID: (pid) => set({ pid }),
}));