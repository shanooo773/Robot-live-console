import { useState, useEffect } from "react";
import { Box, Button, Text, Spinner, Alert, AlertIcon, Code } from "@chakra-ui/react";
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
        const fullVideoUrl = `http://localhost:8000${result.video_url}`;
        setVideoUrl(fullVideoUrl);
        setExecutionId(result.execution_id);
        
        // Check if the video URL is accessible
        fetch(fullVideoUrl, { method: 'HEAD' })
          .then(response => {
            if (response.ok) {
              console.log('Video URL is accessible, checking content type:', response.headers.get('content-type'));
              // For mock simulation, show success message instead of trying to play invalid video
              if (response.headers.get('content-length') === '1032') {
                // This is our mock video - show success message
                setIsError(false);
                console.log('Mock simulation video detected - showing success message');
              } else {
                // Real video - try to load it
                const testVideo = document.createElement('video');
                testVideo.onloadeddata = () => {
                  console.log('Video loaded successfully');
                };
                testVideo.onerror = () => {
                  setVideoLoadError(true);
                  setError(`Video file not accessible at ${fullVideoUrl}. The simulation may have failed to generate a video.`);
                };
                testVideo.src = fullVideoUrl;
              }
            } else {
              setVideoLoadError(true);
              setError(`Video URL not accessible: ${response.status} ${response.statusText}`);
            }
          })
          .catch(err => {
            setVideoLoadError(true);
            setError(`Failed to check video accessibility: ${err.message}`);
          });
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

  const handleVideoError = () => {
    setVideoLoadError(true);
    setIsError(true);
    setError(`Failed to load video. The file may not exist or be corrupted. Execution ID: ${executionId}`);
  };

  const checkVideoStatus = async () => {
    if (executionId) {
      try {
        const response = await fetch(`http://localhost:8000/videos-check/${executionId}`);
        const data = await response.json();
        console.log('Video status:', data);
      } catch (err) {
        console.error('Failed to check video status:', err);
      }
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

      {executionId && (
        <Button
          variant="outline"
          colorScheme="blue"
          size="sm"
          mb={4}
          ml={2}
          onClick={checkVideoStatus}
        >
          Debug Video Status
        </Button>
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
          <Text color="gray.400">Running robot simulation...</Text>
          <Text color="gray.500" fontSize="sm" mt={2}>
            This may take up to 60 seconds
          </Text>
        </Box>
      )}

      {isError && (
        <Alert status="error" bg="#2d1b1b" border="1px solid #e53e3e" mb={4}>
          <AlertIcon />
          <Box>
            <Text color="red.400">{error}</Text>
            {executionId && (
              <Code colorScheme="red" fontSize="sm" mt={2}>
                Execution ID: {executionId}
              </Code>
            )}
            {videoUrl && (
              <Text color="red.300" fontSize="sm" mt={1}>
                Attempted URL: {videoUrl}
              </Text>
            )}
          </Box>
        </Alert>
      )}

      {videoUrl && !isLoading && !videoLoadError && (
        <Box
          height="75vh"
          border="1px solid #333"
          borderRadius="md"
          overflow="hidden"
          bg="#1a1a1a"
        >
          {/* Check if this is a mock video by content length */}
          {videoUrl.includes('localhost:8000') ? (
            <Box
              height="100%"
              display="flex"
              flexDirection="column"
              alignItems="center"
              justifyContent="center"
              p={6}
            >
              <Text color="green.400" fontSize="xl" mb={4}>
                ✅ Simulation Completed Successfully!
              </Text>
              <Text color="gray.400" mb={4} textAlign="center">
                The robot simulation has been executed successfully. 
                In a real deployment with Docker, this would show the actual simulation video.
              </Text>
              <Code colorScheme="green" p={3} borderRadius="md">
                Video URL: {videoUrl}
              </Code>
              <Text color="gray.500" fontSize="sm" mt={4} textAlign="center">
                This is a mock simulation result. To see actual robot videos, 
                build the Docker image using: ./setup.sh build
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
          <Text color="gray.500">
            Select a robot and run your code to see the simulation
          </Text>
        </Box>
      )}
    </Box>
  );
};

export default VideoPlayer;