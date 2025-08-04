import { useState } from "react";
import { 
  Box, 
  Button, 
  Text, 
  VStack, 
  Alert, 
  AlertIcon,
  Code,
  Accordion,
  AccordionItem,
  AccordionButton,
  AccordionPanel,
  AccordionIcon,
  Badge
} from "@chakra-ui/react";
import { 
  getBackendStatus, 
  getDockerStatus, 
  getVideosDebugInfo,
  checkVideoExists,
  cleanupContainers
} from "../api";

const DebugPanel = () => {
  const [debugData, setDebugData] = useState({});
  const [isLoading, setIsLoading] = useState(false);
  const [testExecutionId, setTestExecutionId] = useState("");
  const [isCleaningUp, setIsCleaningUp] = useState(false);

  const runDiagnostics = async () => {
    setIsLoading(true);
    const results = {};

    try {
      // Test backend status
      try {
        results.backend = await getBackendStatus();
        results.backendStatus = "✅ Connected";
      } catch (error) {
        results.backend = { error: error.message };
        results.backendStatus = "❌ Failed";
      }

      // Test Docker status
      try {
        results.docker = await getDockerStatus();
        results.dockerStatus = results.docker.status === "Docker is running" ? "✅ Available" : "⚠️ Issues";
      } catch (error) {
        results.docker = { error: error.message };
        results.dockerStatus = "❌ Failed";
      }

      // Test videos debug info
      try {
        results.videos = await getVideosDebugInfo();
        results.videosStatus = results.videos.videos_directory_exists ? "✅ Ready" : "❌ Missing";
      } catch (error) {
        results.videos = { error: error.message };
        results.videosStatus = "❌ Failed";
      }

      setDebugData(results);
    } catch (error) {
      setDebugData({ error: error.message });
    } finally {
      setIsLoading(false);
    }
  };

  const testVideoAccess = async () => {
    if (!testExecutionId) return;
    
    try {
      const result = await checkVideoExists(testExecutionId);
      setDebugData(prev => ({ ...prev, videoTest: result }));
    } catch (error) {
      setDebugData(prev => ({ 
        ...prev, 
        videoTest: { error: error.message } 
      }));
    }
  };

  const handleCleanupContainers = async () => {
    setIsCleaningUp(true);
    try {
      const result = await cleanupContainers();
      setDebugData(prev => ({ ...prev, cleanup: result }));
    } catch (error) {
      setDebugData(prev => ({ 
        ...prev, 
        cleanup: { success: false, error: error.message } 
      }));
    } finally {
      setIsCleaningUp(false);
    }
  };

  return (
    <Box bg="#1a1a1a" p={6} borderRadius="md" border="1px solid #333">
      <Text fontSize="xl" mb={4} color="blue.400">🔧 System Diagnostics</Text>
      
      <VStack spacing={4} align="stretch">
        <Button 
          onClick={runDiagnostics} 
          isLoading={isLoading}
          colorScheme="blue"
          variant="outline"
        >
          Run Full Diagnostics
        </Button>

        <Button 
          onClick={handleCleanupContainers} 
          isLoading={isCleaningUp}
          colorScheme="red"
          variant="outline"
          size="sm"
        >
          Cleanup Old Docker Containers
        </Button>

        {debugData.cleanup && (
          <Alert status={debugData.cleanup.success ? "success" : "error"}>
            <AlertIcon />
            <Box>
              <Text fontWeight="bold">
                Container Cleanup: {debugData.cleanup.success ? "Success" : "Failed"}
              </Text>
              {debugData.cleanup.success && (
                <Text fontSize="sm">
                  Removed {debugData.cleanup.total_removed} containers
                </Text>
              )}
              {debugData.cleanup.error && (
                <Text fontSize="sm" color="red.300">
                  {debugData.cleanup.error}
                </Text>
              )}
            </Box>
          </Alert>
        )}

        {debugData.backendStatus && (
          <Alert status={debugData.backendStatus.includes("✅") ? "success" : "error"}>
            <AlertIcon />
            <Text>Backend: {debugData.backendStatus}</Text>
          </Alert>
        )}

        {debugData.dockerStatus && (
          <Alert status={debugData.dockerStatus.includes("✅") ? "success" : 
                        debugData.dockerStatus.includes("⚠️") ? "warning" : "error"}>
            <AlertIcon />
            <Text>Docker: {debugData.dockerStatus}</Text>
          </Alert>
        )}

        {debugData.videosStatus && (
          <Alert status={debugData.videosStatus.includes("✅") ? "success" : "error"}>
            <AlertIcon />
            <Text>Videos Directory: {debugData.videosStatus}</Text>
          </Alert>
        )}

        <Accordion allowToggle>
          <AccordionItem>
            <h2>
              <AccordionButton>
                <Box flex="1" textAlign="left">
                  Detailed Results
                </Box>
                <AccordionIcon />
              </AccordionButton>
            </h2>
            <AccordionPanel pb={4}>
              <VStack spacing={4} align="stretch">
                {debugData.backend && (
                  <Box>
                    <Text fontWeight="bold" color="green.400">Backend Response:</Text>
                    <Code p={2} display="block" whiteSpace="pre">
                      {JSON.stringify(debugData.backend, null, 2)}
                    </Code>
                  </Box>
                )}

                {debugData.docker && (
                  <Box>
                    <Text fontWeight="bold" color="blue.400">Docker Status:</Text>
                    <Code p={2} display="block" whiteSpace="pre">
                      {JSON.stringify(debugData.docker, null, 2)}
                    </Code>
                  </Box>
                )}

                {debugData.videos && (
                  <Box>
                    <Text fontWeight="bold" color="purple.400">Videos Debug Info:</Text>
                    <Code p={2} display="block" whiteSpace="pre">
                      {JSON.stringify(debugData.videos, null, 2)}
                    </Code>
                    {debugData.videos.video_files && debugData.videos.video_files.length > 0 && (
                      <Box mt={2}>
                        <Text fontSize="sm" color="gray.400">Available Videos:</Text>
                        {debugData.videos.video_files.map((file, index) => (
                          <Badge key={index} colorScheme="green" mr={2}>
                            {file.name} ({file.size_bytes} bytes)
                          </Badge>
                        ))}
                      </Box>
                    )}
                  </Box>
                )}
              </VStack>
            </AccordionPanel>
          </AccordionItem>
        </Accordion>

        <Box p={4} bg="#2a2a2a" borderRadius="md">
          <Text fontSize="md" mb={2} color="yellow.400">Test Video Access:</Text>
          <VStack spacing={2}>
            <Box>
              <input
                type="text"
                placeholder="Enter execution ID (e.g., test)"
                value={testExecutionId}
                onChange={(e) => setTestExecutionId(e.target.value)}
                style={{
                  background: "#1a1a1a",
                  border: "1px solid #333",
                  borderRadius: "4px",
                  padding: "8px",
                  color: "white",
                  width: "300px"
                }}
              />
            </Box>
            <Button 
              size="sm" 
              onClick={testVideoAccess}
              disabled={!testExecutionId}
              colorScheme="yellow"
              variant="outline"
            >
              Check Video
            </Button>
            {debugData.videoTest && (
              <Code p={2} display="block" whiteSpace="pre" fontSize="sm">
                {JSON.stringify(debugData.videoTest, null, 2)}
              </Code>
            )}
          </VStack>
        </Box>

        <Box p={4} bg="#2a2a2a" borderRadius="md">
          <Text fontSize="md" mb={2} color="orange.400">🎯 Debugging Checklist:</Text>
          <VStack align="start" spacing={1} fontSize="sm">
            <Text>1. ✅ Backend running at http://localhost:8000</Text>
            <Text>2. ✅ FastAPI serving static files from /videos</Text>
            <Text>3. ✅ Videos directory exists and is writable</Text>
            <Text>4. ⚠️ Docker simulation container needs to be built</Text>
            <Text>5. 🔄 Test video generation and serving flow</Text>
          </VStack>
        </Box>
      </VStack>
    </Box>
  );
};

export default DebugPanel;