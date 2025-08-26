import { useState, useEffect } from "react";
import { Box, Button, Text, Spinner, Alert, AlertIcon, VStack, HStack } from "@chakra-ui/react";
import { motion } from "framer-motion";
import { executeRobotCode } from "../api";

const MotionBox = motion(Box);
const MotionButton = motion(Button);

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
    <VStack w="100%" spacing={6} align="start">
      <HStack w="full" justify="space-between" align="center">
        <HStack align="center" gap={3}>
          <Box fontSize="lg">🎬</Box>
          <Text fontSize="lg" color="white" fontWeight="bold">
            Robot Simulation
          </Text>
        </HStack>
      </HStack>
      
      <MotionButton
        variant="gradient"
        isLoading={isLoading}
        loadingText="🚀 Running Simulation..."
        onClick={runCode}
        size="lg"
        w="full"
        py={6}
        h="auto"
        whileHover={{ scale: 1.02 }}
        whileTap={{ scale: 0.98 }}
      >
        ▶️ Run Code
      </MotionButton>

      {isError && (
        <MotionBox
          w="full"
          initial={{ opacity: 0, scale: 0.9 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.3 }}
        >
          <Alert 
            status="error" 
            borderRadius="xl"
            bg="rgba(244, 63, 94, 0.1)"
            border="1px solid rgba(244, 63, 94, 0.3)"
            color="red.300"
          >
            <AlertIcon color="red.400" />
            <Box>
              <Text fontWeight="bold">Execution Failed</Text>
              <Text fontSize="sm">{error}</Text>
            </Box>
          </Alert>
        </MotionBox>
      )}

      <MotionBox
        w="full"
        height="75vh"
        borderRadius="xl"
        overflow="hidden"
        border="1px solid"
        borderColor="rgba(255, 255, 255, 0.1)"
        bg="rgba(0, 0, 0, 0.4)"
        backdropFilter="blur(10px)"
        position="relative"
        whileHover={{ 
          borderColor: "rgba(56, 189, 248, 0.3)",
          boxShadow: "0 0 30px rgba(56, 189, 248, 0.1)"
        }}
        transition={{ duration: 0.3 }}
      >
        {isLoading && (
          <MotionBox
            position="absolute"
            top="0"
            left="0"
            right="0"
            bottom="0"
            display="flex"
            flexDirection="column"
            alignItems="center"
            justifyContent="center"
            bg="rgba(0, 0, 0, 0.8)"
            backdropFilter="blur(5px)"
            zIndex="10"
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
          >
            <MotionBox
              animate={{ rotate: 360 }}
              transition={{ duration: 2, repeat: Infinity, ease: "linear" }}
              mb={6}
            >
              <Spinner size="xl" color="brand.400" thickness="4px" />
            </MotionBox>
            <Text color="gray.300" fontSize="lg" mb={2} fontWeight="600">
              Running Robot Simulation...
            </Text>
            <Text color="gray.500" fontSize="sm" textAlign="center" maxW="md">
              Your code is being executed in the robot simulation environment.
              This may take a few moments.
            </Text>
          </MotionBox>
        )}

        {videoUrl && !isLoading && (
          <MotionBox
            width="100%"
            height="100%"
            initial={{ opacity: 0, scale: 0.95 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ duration: 0.5 }}
          >
            {videoLoadError ? (
              <Box
                height="100%"
                display="flex"
                flexDirection="column"
                alignItems="center"
                justifyContent="center"
                p={8}
              >
                <Text fontSize="4xl" mb={4}>⚠️</Text>
                <Text color="yellow.400" fontSize="lg" mb={2} fontWeight="600">
                  Video Unavailable
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
                style={{ 
                  objectFit: "contain",
                  borderRadius: "12px"
                }}
                onError={handleVideoError}
                onLoadStart={() => console.log('Video load started')}
                onCanPlay={() => console.log('Video can play')}
              >
                <source src={videoUrl} type="video/mp4" />
                Your browser does not support the video tag.
              </video>
            )}
          </MotionBox>
        )}

        {!videoUrl && !isLoading && !isError && (
          <Box
            height="100%"
            display="flex"
            flexDirection="column"
            alignItems="center"
            justifyContent="center"
            p={8}
          >
            <Text fontSize="6xl" mb={6} opacity="0.3">🤖</Text>
            <Text color="gray.400" textAlign="center" fontSize="lg" fontWeight="600">
              Ready to run simulation
            </Text>
            <Text color="gray.500" textAlign="center" mt={2}>
              Select a robot and run your code to see the simulation
            </Text>
          </Box>
        )}
      </MotionBox>
    </VStack>
  );
};

export default VideoPlayer;