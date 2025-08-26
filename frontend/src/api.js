import axios from "axios";

// Admin backend API (authentication and booking)
const ADMIN_API = axios.create({
  baseURL: "http://localhost:8000",
});

// Simulation service API (robot simulation)
const SIMULATION_API = axios.create({
  baseURL: "http://localhost:8001",
});

// Authentication API
export const registerUser = async (userData) => {
  const response = await ADMIN_API.post("/auth/register", userData);
  return response.data;
};

export const loginUser = async (credentials) => {
  const response = await ADMIN_API.post("/auth/login", credentials);
  return response.data;
};

export const getCurrentUser = async (token) => {
  const response = await ADMIN_API.get("/auth/me", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

// Booking API
export const createBooking = async (bookingData, token) => {
  const response = await ADMIN_API.post("/bookings", bookingData, {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const getUserBookings = async (token) => {
  const response = await ADMIN_API.get("/bookings", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const getBookingSchedule = async (startDate, endDate) => {
  const response = await ADMIN_API.get(`/bookings/schedule?start_date=${startDate}&end_date=${endDate}`);
  return response.data;
};

// Admin API
export const getAllUsers = async (token) => {
  const response = await ADMIN_API.get("/admin/users", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const getAllBookings = async (token) => {
  const response = await ADMIN_API.get("/bookings/all", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const getAdminStats = async (token) => {
  const response = await ADMIN_API.get("/admin/stats", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const updateBookingStatus = async (bookingId, status, token) => {
  const response = await ADMIN_API.put(`/bookings/${bookingId}`, { status }, {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const deleteBooking = async (bookingId, token) => {
  const response = await ADMIN_API.delete(`/bookings/${bookingId}`, {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

// Robot simulation API
export const executeRobotCode = async (code, robotType) => {
  const response = await SIMULATION_API.post("/run-code", {
    code: code,
    robot_type: robotType,
  });
  return response.data;
};

export const getAvailableRobots = async () => {
  const response = await SIMULATION_API.get("/robots");
  return response.data;
};

// Health check API
export const getSystemHealth = async () => {
  const response = await ADMIN_API.get("/health");
  return response.data;
};

export const getServiceStatus = async () => {
  const response = await ADMIN_API.get("/health/services");
  return response.data;
};

export const getAvailableFeatures = async () => {
  const response = await ADMIN_API.get("/health/features");
  return response.data;
};

// Simulation service health check
export const getSimulationHealth = async () => {
  const response = await SIMULATION_API.get("/health");
  return response.data;
};
