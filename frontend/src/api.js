import axios from "axios";

const API = axios.create({
  baseURL: "http://localhost:8000",
});

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

export const getVideosDebugInfo = async () => {
  const response = await API.get("/videos-debug");
  return response.data;
};

export const checkVideoExists = async (executionId) => {
  const response = await API.get(`/videos-check/${executionId}`);
  return response.data;
};

export const getDockerStatus = async () => {
  const response = await API.get("/docker-status");
  return response.data;
};

export const getBackendStatus = async () => {
  const response = await API.get("/status");
  return response.data;
};
