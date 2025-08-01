import { useState } from "react";
import { Box, Button, Text, Spinner, Alert, AlertIcon } from "@chakra-ui/react";
import { executeRobotCode } from "../api";

const VideoPlayer = ({ editorRef, robot }) => {
  const [videoUrl, setVideoUrl] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [isError, setIsError] = useState(false);
  const [error, setError] = useState("");

  const runCode = async () => {
    const sourceCode = editorRef.current.getValue();
    if (!sourceCode) return;

    if (!robot) {
      setIsError(true);
      setError("Please select a robot type first");
      return;
    }

    try {
      setIsLoading(true);
      setIsError(false);
      setVideoUrl("");
      setError("");

      const result = await executeRobotCode(sourceCode, robot);
      
      if (result.success && result.video_url) {
        setVideoUrl(`http://localhost:8000${result.video_url}`);
      } else {
        setIsError(true);
        setError(result.error || "Failed to run simulation");
      }
    } catch (err) {
      setIsError(true);
      setError(err.message || "Connection error. Make sure the backend is running.");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Box w="50%">
      <Text mb={2} fontSize="lg">
        Robot Simulation:
      </Text>
      <Button
        variant="outline"
        colorScheme="green"
        mb={4}
        onClick={runCode}
        isLoading={isLoading}
        loadingText="Running Simulation..."
        disabled={!robot}
      >
        Run Code
      </Button>

      {isLoading && (
        <Box
          height="75vh"
          display="flex"
          flexDirection="column"
          alignItems="center"
          justifyContent="center"
          border="1px solid #333"
          borderRadius="md"
          bg="#1a1a1a"
        >
          <Spinner size="xl" color="blue.400" mb={4} />
          <Text color="gray.400">Running robot simulation...</Text>
          <Text color="gray.500" fontSize="sm" mt={2}>
            This may take up to 60 seconds
          </Text>
        </Box>
      )}

      {isError && (
        <Alert status="error" bg="#2d1b1b" border="1px solid #e53e3e">
          <AlertIcon />
          <Text color="red.400">{error}</Text>
        </Alert>
      )}

      {videoUrl && !isLoading && (
        <Box
          height="75vh"
          border="1px solid #333"
          borderRadius="md"
          overflow="hidden"
          bg="#1a1a1a"
        >
          <video
            width="100%"
            height="100%"
            controls
            autoPlay
            style={{ objectFit: "contain" }}
          >
            <source src={videoUrl} type="video/mp4" />
            Your browser does not support the video tag.
          </video>
        </Box>
      )}

      {!videoUrl && !isLoading && !isError && (
        <Box
          height="75vh"
          display="flex"
          alignItems="center"
          justifyContent="center"
          border="1px solid #333"
          borderRadius="md"
          bg="#1a1a1a"
        >
          <Text color="gray.500">
            Select a robot and run your code to see the simulation
          </Text>
        </Box>
      )}
    </Box>
  );
};

export default VideoPlayer;