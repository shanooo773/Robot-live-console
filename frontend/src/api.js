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
