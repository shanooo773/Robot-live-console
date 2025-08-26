import { useState } from "react";
import { 
  Box, 
  Button, 
  Text, 
  VStack, 
  HStack,
  Badge,
  Accordion,
  AccordionItem,
  AccordionButton,
  AccordionPanel,
  AccordionIcon,
  Code,
  Alert,
  AlertIcon,
  useToast
} from "@chakra-ui/react";
import { executeRobotCode } from "../api";

const DebugPanel = () => {
  const [systemStatus, setSystemStatus] = useState(null);
  const [isLoading, setIsLoading] = useState(false);
  const toast = useToast();

  const checkSystemStatus = async () => {
    setIsLoading(true);
    try {
      const responses = await Promise.all([
        fetch('http://localhost:8000/status').then(r => r.json()),
        fetch('http://localhost:8000/videos-debug').then(r => r.json()),
        fetch('http://localhost:8000/docker-status').then(r => r.json()),
        fetch('http://localhost:8000/debug-info').then(r => r.json())
      ]);

      setSystemStatus({
        status: responses[0],
        videos: responses[1],
        docker: responses[2],
        debug: responses[3]
      });

      toast({
        title: "System check complete",
        status: "success",
        duration: 2000
      });
    } catch (error) {
      console.error("System check failed:", error);
      toast({
        title: "System check failed",
        description: error.message,
        status: "error",
        duration: 3000
      });
    } finally {
      setIsLoading(false);
    }
  };

  const testVideoGeneration = async () => {
    try {
      const result = await executeRobotCode(
        'print("Debug test - generating video...")',
        'turtlebot'
      );
      
      toast({
        title: "Test completed",
        description: result.success ? "Video generated successfully" : "Video generation failed",
        status: result.success ? "success" : "error",
        duration: 3000
      });

      // Refresh system status after test
      await checkSystemStatus();
    } catch (error) {
      toast({
        title: "Test failed",
        description: error.message,
        status: "error",
        duration: 3000
      });
    }
  };

  const StatusBadge = ({ condition, label }) => (
    <Badge colorScheme={condition ? "green" : "red"}>
      {condition ? "✅" : "❌"} {label}
    </Badge>
  );

  return (
    <Box p={6} bg="gray.800" borderRadius="md" border="1px solid" borderColor="gray.600">
      <VStack spacing={4} align="stretch">
        <Text fontSize="xl" fontWeight="bold" color="white">
          🛠️ System Diagnostics
        </Text>

        <HStack>
          <Button 
            colorScheme="blue" 
            onClick={checkSystemStatus}
            isLoading={isLoading}
            loadingText="Checking..."
          >
            Run Full Diagnostics
          </Button>
          <Button 
            colorScheme="green" 
            onClick={testVideoGeneration}
          >
            Test Video Generation
          </Button>
        </HStack>

        {systemStatus && (
          <Accordion allowToggle>
            <AccordionItem>
              <AccordionButton>
                <Box flex="1" textAlign="left">
                  <Text fontWeight="bold" color="white">System Overview</Text>
                </Box>
                <AccordionIcon />
              </AccordionButton>
              <AccordionPanel pb={4}>
                <VStack spacing={2} align="start">
                  <StatusBadge 
                    condition={systemStatus.status?.status === "Backend is running"} 
                    label="Backend Running"
                  />
                  <StatusBadge 
                    condition={systemStatus.videos?.videos_directory_exists} 
                    label="Videos Directory"
                  />
                  <StatusBadge 
                    condition={systemStatus.docker?.docker_ping_successful} 
                    label="Docker Connected"
                  />
                  <StatusBadge 
                    condition={systemStatus.docker?.robot_simulation_image_available} 
                    label="Simulation Image"
                  />
                </VStack>
              </AccordionPanel>
            </AccordionItem>

            <AccordionItem>
              <AccordionButton>
                <Box flex="1" textAlign="left">
                  <Text fontWeight="bold" color="white">Backend Status</Text>
                </Box>
                <AccordionIcon />
              </AccordionButton>
              <AccordionPanel pb={4}>
                <Code display="block" whiteSpace="pre" p={3} fontSize="sm">
                  {JSON.stringify(systemStatus.status, null, 2)}
                </Code>
              </AccordionPanel>
            </AccordionItem>

            <AccordionItem>
              <AccordionButton>
                <Box flex="1" textAlign="left">
                  <Text fontWeight="bold" color="white">Videos Directory</Text>
                </Box>
                <AccordionIcon />
              </AccordionButton>
              <AccordionPanel pb={4}>
                <Code display="block" whiteSpace="pre" p={3} fontSize="sm">
                  {JSON.stringify(systemStatus.videos, null, 2)}
                </Code>
                {systemStatus.videos?.videos_list?.length > 0 && (
                  <Box mt={3}>
                    <Text color="gray.300" mb={2}>Recent Videos:</Text>
                    {systemStatus.videos.videos_list.map(video => (
                      <Text key={video} color="gray.400" fontSize="sm">
                        <Button 
                          as="a" 
                          href={`http://localhost:8000/videos/${video}`}
                          target="_blank"
                          size="sm"
                          variant="link"
                          color="blue.300"
                        >
                          {video}
                        </Button>
                      </Text>
                    ))}
                  </Box>
                )}
              </AccordionPanel>
            </AccordionItem>

            <AccordionItem>
              <AccordionButton>
                <Box flex="1" textAlign="left">
                  <Text fontWeight="bold" color="white">Docker Status</Text>
                </Box>
                <AccordionIcon />
              </AccordionButton>
              <AccordionPanel pb={4}>
                {!systemStatus.docker?.robot_simulation_image_available && (
                  <Alert status="warning" mb={3}>
                    <AlertIcon />
                    <Box>
                      <Text fontWeight="bold">Docker Image Missing</Text>
                      <Text fontSize="sm">
                        The robot-simulation:latest image is not available. 
                        The system will use mock simulation mode.
                      </Text>
                    </Box>
                  </Alert>
                )}
                <Code display="block" whiteSpace="pre" p={3} fontSize="sm">
                  {JSON.stringify(systemStatus.docker, null, 2)}
                </Code>
              </AccordionPanel>
            </AccordionItem>

            <AccordionItem>
              <AccordionButton>
                <Box flex="1" textAlign="left">
                  <Text fontWeight="bold" color="white">Full Debug Info</Text>
                </Box>
                <AccordionIcon />
              </AccordionButton>
              <AccordionPanel pb={4}>
                <Code display="block" whiteSpace="pre" p={3} fontSize="sm" maxH="400px" overflowY="auto">
                  {JSON.stringify(systemStatus.debug, null, 2)}
                </Code>
              </AccordionPanel>
            </AccordionItem>
          </Accordion>
        )}

        <Box>
          <Text fontSize="sm" color="gray.400">
            💡 Use this panel to diagnose connectivity issues between frontend and backend,
            check Docker status, and verify video generation is working properly.
          </Text>
        </Box>
      </VStack>
    </Box>
  );
};

export default DebugPanel;