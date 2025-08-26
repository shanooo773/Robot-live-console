import { useState, useEffect } from "react";
import { Box, Button, Text, Spinner, Alert, AlertIcon, Icon, keyframes } from "@chakra-ui/react";
import { motion } from "framer-motion";
import { FiPlay, FiRefreshCw, FiVideo } from "react-icons/fi";
import { executeRobotCode } from "../api";

const MotionBox = motion(Box);
const MotionButton = motion(Button);

// Animation keyframes
const rotate = keyframes`
  from { transform: rotate(0deg); }
  to { transform: rotate(360deg); }
`;

const glow = keyframes`
  0% { box-shadow: 0 0 20px rgba(67, 233, 123, 0.3); }
  50% { box-shadow: 0 0 30px rgba(67, 233, 123, 0.6); }
  100% { box-shadow: 0 0 20px rgba(67, 233, 123, 0.3); }
`;

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
    <MotionBox w="100%">
      <MotionButton
        variant="success"
        mb={6}
        isLoading={isLoading}
        loadingText="Running Simulation..."
        onClick={runCode}
        size="lg"
        w="full"
        borderRadius="xl"
        leftIcon={<Icon as={isLoading ? FiRefreshCw : FiPlay} />}
        animation={isLoading ? `${rotate} 1s linear infinite` : undefined}
        _hover={{
          animation: `${glow} 2s ease-in-out infinite`,
        }}
        whileHover={{ scale: 1.02 }}
        whileTap={{ scale: 0.98 }}
      >
        {isLoading ? "Running Simulation..." : "Run Code"}
      </MotionButton>

      {isError && (
        <MotionBox
          initial={{ opacity: 0, y: -10 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.3 }}
          mb={4}
        >
          <Alert 
            status="error" 
            borderRadius="xl"
            bg="rgba(245, 87, 108, 0.1)"
            border="1px solid rgba(245, 87, 108, 0.3)"
          >
            <AlertIcon color="red.300" />
            <Box>
              <Text fontWeight="bold" color="red.300">Execution Failed</Text>
              <Text color="red.200">{error}</Text>
            </Box>
          </Alert>
        </MotionBox>
      )}

      {isLoading && (
        <MotionBox
          height="75vh"
          display="flex"
          flexDirection="column"
          alignItems="center"
          justifyContent="center"
          borderRadius="xl"
          bg="rgba(255, 255, 255, 0.02)"
          border="1px solid rgba(255, 255, 255, 0.1)"
          backdropFilter="blur(10px)"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.5 }}
        >
          <Spinner 
            size="xl" 
            color="blue.400" 
            mb={6}
            thickness="4px"
          />
          <Text color="gray.300" fontSize="lg" mb={3} fontWeight="medium">
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
          height="75vh"
          borderRadius="xl"
          overflow="hidden"
          border="1px solid rgba(255, 255, 255, 0.1)"
          bg="rgba(0, 0, 0, 0.5)"
          backdropFilter="blur(10px)"
          initial={{ opacity: 0, scale: 0.95 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.5 }}
          whileHover={{ 
            boxShadow: "0 15px 40px rgba(67, 233, 123, 0.2)",
            transition: { duration: 0.3 }
          }}
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
              <Icon as={FiVideo} color="red.300" boxSize={16} mb={4} />
              <Text color="red.300" fontSize="lg" mb={2} fontWeight="bold">
                Video Load Error
              </Text>
              <Text color="gray.400" textAlign="center" maxW="md">
                The simulation video could not be loaded. This might be due to network issues or the simulation is still processing.
              </Text>
              <Button
                variant="glass"
                mt={4}
                onClick={() => {
                  setVideoLoadError(false);
                  // Force video reload by adding timestamp
                  const timestamp = Date.now();
                  setVideoUrl(`${videoUrl}?t=${timestamp}`);
                }}
                leftIcon={<Icon as={FiRefreshCw} />}
              >
                Retry
              </Button>
            </Box>
          ) : (
            <video
              width="100%"
              height="100%"
              controls
              autoPlay
              muted
              onError={handleVideoError}
              style={{
                objectFit: 'contain',
                backgroundColor: '#000'
              }}
            >
              <source src={videoUrl} type="video/mp4" />
              Your browser does not support the video tag.
            </video>
          )}
        </MotionBox>
      )}

      {!videoUrl && !isLoading && !isError && (
        <MotionBox
          height="75vh"
          display="flex"
          flexDirection="column"
          alignItems="center"
          justifyContent="center"
          borderRadius="xl"
          bg="rgba(255, 255, 255, 0.02)"
          border="2px dashed rgba(255, 255, 255, 0.2)"
          backdropFilter="blur(10px)"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.5 }}
        >
          <Icon as={FiPlay} color="gray.400" boxSize={16} mb={4} />
          <Text color="gray.300" fontSize="lg" mb={2} fontWeight="medium">
            Ready for Simulation
          </Text>
          <Text color="gray.500" textAlign="center" maxW="md">
            Click "Run Code" to execute your robot program and see the simulation results
          </Text>
        </MotionBox>
      )}
    </MotionBox>
  );
};

export default VideoPlayer;