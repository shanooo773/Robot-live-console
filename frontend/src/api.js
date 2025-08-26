import axios from "axios";

const API = axios.create({
  baseURL: "http://localhost:8000",
});

// Authentication API
export const registerUser = async (userData) => {
  const response = await API.post("/auth/register", userData);
  return response.data;
};

export const loginUser = async (credentials) => {
  const response = await API.post("/auth/login", credentials);
  return response.data;
};

export const getCurrentUser = async (token) => {
  const response = await API.get("/auth/me", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

// Booking API
export const createBooking = async (bookingData, token) => {
  const response = await API.post("/bookings", bookingData, {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const getUserBookings = async (token) => {
  const response = await API.get("/bookings", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const getBookingSchedule = async (startDate, endDate) => {
  const response = await API.get(`/bookings/schedule?start_date=${startDate}&end_date=${endDate}`);
  return response.data;
};

// Admin API
export const getAllUsers = async (token) => {
  const response = await API.get("/admin/users", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const getAllBookings = async (token) => {
  const response = await API.get("/bookings/all", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const getAdminStats = async (token) => {
  const response = await API.get("/admin/stats", {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const updateBookingStatus = async (bookingId, status, token) => {
  const response = await API.put(`/bookings/${bookingId}`, { status }, {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

export const deleteBooking = async (bookingId, token) => {
  const response = await API.delete(`/bookings/${bookingId}`, {
    headers: { Authorization: `Bearer ${token}` }
  });
  return response.data;
};

// Robot simulation API
export const executeRobotCode = async (code, robotType) => {
  const response = await API.post("/run-code", {
    code: code,
    robot_type: robotType,
  });
  return response.data;
};

export const getAvailableRobots = async () => {
  const response = await API.get("/robots");
  return response.data;
};
