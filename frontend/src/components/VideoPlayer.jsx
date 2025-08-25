import { useState, useEffect } from "react";
import { Box, Button, Text, Spinner, Alert, AlertIcon } from "@chakra-ui/react";
import { executeRobotCode } from "../api";

const VideoPlayer = ({ editorRef, robot, codeValue }) => {
  const [videoUrl, setVideoUrl] = useState("");
  const [isLoading, setIsLoading] = useState(false);
  const [isError, setIsError] = useState(false);
  const [error, setError] = useState("");
  const [executionId, setExecutionId] = useState("");
  const [videoLoadError, setVideoLoadError] = useState(false);

  // Reset video load error when videoUrl changes
  useEffect(() => {
    setVideoLoadError(false);
  }, [videoUrl]);

  const runCode = async () => {
    // Get source code from editor or fallback to prop value
    let sourceCode = "";
    
    if (editorRef.current) {
      try {
        sourceCode = editorRef.current.getValue();
      } catch (err) {
        console.warn("Could not get code from editor, using fallback value");
        sourceCode = codeValue;
      }
    } else {
      sourceCode = codeValue;
    }

    if (!sourceCode || sourceCode.trim() === "") {
      setIsError(true);
      setError("Please enter some code to run");
      return;
    }

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
      setExecutionId("");
      setVideoLoadError(false);

      const result = await executeRobotCode(sourceCode, robot);
      
      if (result.success && result.video_url) {
        setVideoUrl(result.video_url);
        setExecutionId(result.execution_id || "");
      } else {
        setIsError(true);
        setError(result.error || "Failed to execute code");
      }
    } catch (error) {
      console.error("Execution error:", error);
      setIsError(true);
      setError(error.response?.data?.detail || error.message || "An unexpected error occurred");
    } finally {
      setIsLoading(false);
    }
  };

  const handleVideoError = () => {
    console.warn("Video failed to load");
    setVideoLoadError(true);
  };

  return (
    <Box w="100%">
      <Text mb={2} fontSize="lg" color="white" fontWeight="bold">
        Robot Simulation
      </Text>
      
      <Button
        colorScheme="green"
        mb={4}
        isLoading={isLoading}
        loadingText="Running Simulation..."
        onClick={runCode}
        size="lg"
        w="full"
      >
        Run Code
      </Button>

      {isError && (
        <Alert status="error" mb={4}>
          <AlertIcon />
          <Box>
            <Text fontWeight="bold">Execution Failed</Text>
            <Text fontSize="sm">{error}</Text>
          </Box>
        </Alert>
      )}

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
          <Text color="gray.300" fontSize="lg" mb={2}>
            Running Robot Simulation...
          </Text>
          <Text color="gray.500" fontSize="sm" textAlign="center" maxW="md">
            Your code is being executed in the robot simulation environment.
            This may take a few moments.
          </Text>
        </Box>
      )}

      {videoUrl && !isLoading && (
        <Box
          height="75vh"
          border="1px solid #333"
          borderRadius="md"
          bg="#1a1a1a"
          overflow="hidden"
        >
          {videoLoadError ? (
            <Box
              height="100%"
              display="flex"
              flexDirection="column"
              alignItems="center"
              justifyContent="center"
              p={6}
            >
              <Text color="yellow.400" fontSize="lg" mb={2}>
                ⚠️ Video Unavailable
              </Text>
              <Text color="gray.300" textAlign="center" mb={4}>
                The simulation completed successfully, but the video couldn't be loaded.
                This might be due to the simulation running in mock mode.
              </Text>
              <Text color="gray.400" fontSize="sm" textAlign="center">
                Execution ID: {executionId}
              </Text>
            </Box>
          ) : (
            <video
              width="100%"
              height="100%"
              controls
              autoPlay
              style={{ objectFit: "contain" }}
              onError={handleVideoError}
              onLoadStart={() => console.log('Video load started')}
              onCanPlay={() => console.log('Video can play')}
            >
              <source src={videoUrl} type="video/mp4" />
              Your browser does not support the video tag.
            </video>
          )}
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
          <Text color="gray.500" textAlign="center">
            Select a robot and run your code to see the simulation
          </Text>
        </Box>
      )}
    </Box>
  );
};

export default VideoPlayer;