import React, { useState } from 'react';
import {
  Box,
  VStack,
  HStack,
  Button,
  Text,
  Input,
  FormControl,
  FormLabel,
  Alert,
  AlertIcon,
  Progress,
  useToast,
  Divider,
  Card,
  CardBody,
  CardHeader,
  Heading,
  Badge,
  Flex,
  Spacer,
  Textarea,
  Accordion,
  AccordionItem,
  AccordionButton,
  AccordionPanel,
  AccordionIcon,
  Code,
  Spinner,
  AlertDialog,
  AlertDialogBody,
  AlertDialogFooter,
  AlertDialogHeader,
  AlertDialogContent,
  AlertDialogOverlay,
  useDisclosure
} from '@chakra-ui/react';
import { FiUpload, FiPlay, FiDownload, FiEye, FiInfo } from 'react-icons/fi';
import { getExecutionLogs } from '../api';

const SimulationUploader = () => {
  const [urdfFile, setUrdfFile] = useState(null);
  const [worldFile, setWorldFile] = useState(null);
  const [duration, setDuration] = useState(10);
  const [uploading, setUploading] = useState(false);
  const [running, setRunning] = useState(false);
  const [uploadResult, setUploadResult] = useState(null);
  const [simulationResult, setSimulationResult] = useState(null);
  const [logs, setLogs] = useState('');
  const [loadingLogs, setLoadingLogs] = useState(false);
  const [simulationError, setSimulationError] = useState(null);
  const { isOpen, onOpen, onClose } = useDisclosure();
  const toast = useToast();

  const handleFileChange = (file, type) => {
    if (type === 'urdf') {
      setUrdfFile(file);
    } else if (type === 'world') {
      setWorldFile(file);
    }
    // Clear previous results when files change
    setUploadResult(null);
    setSimulationResult(null);
    setLogs('');
    setSimulationError(null);
  };

  const uploadFiles = async () => {
    if (!urdfFile || !worldFile) {
      toast({
        title: 'Missing Files',
        description: 'Please select both URDF and World files',
        status: 'warning',
        duration: 3000,
        isClosable: true,
      });
      return;
    }

    setUploading(true);
    const formData = new FormData();
    formData.append('urdf_file', urdfFile);
    formData.append('world_file', worldFile);

    try {
      const response = await fetch('http://localhost:8000/upload-files', {
        method: 'POST',
        body: formData,
      });

      if (!response.ok) {
        const errorData = await response.json().catch(() => ({ detail: response.statusText }));
        throw new Error(errorData.detail || `Upload failed: ${response.statusText}`);
      }

      const result = await response.json();
      setUploadResult(result);
      toast({
        title: 'Files Uploaded Successfully',
        description: `Upload ID: ${result.upload_id}`,
        status: 'success',
        duration: 3000,
        isClosable: true,
      });
    } catch (error) {
      console.error('Upload error:', error);
      toast({
        title: 'Upload Failed',
        description: error.message,
        status: 'error',
        duration: 5000,
        isClosable: true,
      });
    } finally {
      setUploading(false);
    }
  };

  const fetchLogs = async (executionId) => {
    setLoadingLogs(true);
    try {
      const logsData = await getExecutionLogs(executionId);
      setLogs(logsData.logs || 'No logs available');
    } catch (error) {
      console.error('Error fetching logs:', error);
      setLogs(`Error fetching logs: ${error.message}`);
    } finally {
      setLoadingLogs(false);
    }
  };

  const runSimulation = async () => {
    if (!uploadResult) {
      toast({
        title: 'No Files Uploaded',
        description: 'Please upload files first',
        status: 'warning',
        duration: 3000,
        isClosable: true,
      });
      return;
    }

    setRunning(true);
    setSimulationResult(null);
    setLogs('');
    setSimulationError(null);

    try {
      const response = await fetch('http://localhost:8000/simulate', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          urdf_path: uploadResult.urdf_path,
          world_path: uploadResult.world_path,
          duration: duration,
        }),
      });

      const result = await response.json();
      setSimulationResult(result);

      // Always fetch logs after simulation attempt
      if (result.execution_id) {
        await fetchLogs(result.execution_id);
      }

      if (result.success) {
        toast({
          title: 'Simulation Completed',
          description: `Video generated successfully`,
          status: 'success',
          duration: 3000,
          isClosable: true,
        });
      } else {
        // Store detailed error information
        setSimulationError({
          message: result.error,
          executionId: result.execution_id,
          details: result.error_details,
          logsUrl: result.logs_url
        });
        
        toast({
          title: 'Simulation Failed',
          description: 'Click "View Error Details" for more information',
          status: 'error',
          duration: 5000,
          isClosable: true,
        });
      }
    } catch (error) {
      console.error('Simulation error:', error);
      setSimulationError({
        message: error.message,
        networkError: true
      });
      
      toast({
        title: 'Simulation Error',
        description: 'Network or server error occurred',
        status: 'error',
        duration: 5000,
        isClosable: true,
      });
    } finally {
      setRunning(false);
    }
  };

  const downloadVideo = () => {
    if (simulationResult && simulationResult.video_url) {
      const link = document.createElement('a');
      link.href = `http://localhost:8000${simulationResult.video_url}`;
      link.download = `simulation_${simulationResult.execution_id}.mp4`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
    }
  };

  const formatLogOutput = (logText) => {
    if (!logText) return 'No logs available';
    
    // Split logs into lines and add formatting
    const lines = logText.split('\n');
    return lines.map((line, index) => {
      let lineClass = '';
      if (line.includes('ERROR') || line.includes('FAILED')) {
        lineClass = 'error-line';
      } else if (line.includes('WARNING') || line.includes('WARN')) {
        lineClass = 'warning-line';
      } else if (line.includes('SUCCESS') || line.includes('completed successfully')) {
        lineClass = 'success-line';
      }
      
      return (
        <Text 
          key={index} 
          fontSize="xs" 
          fontFamily="monospace"
          color={
            lineClass === 'error-line' ? 'red.300' :
            lineClass === 'warning-line' ? 'yellow.300' :
            lineClass === 'success-line' ? 'green.300' :
            'gray.300'
          }
        >
          {line}
        </Text>
      );
    });
  };

  return (
    <Box maxW="4xl" mx="auto" p={6}>
      <Card bg="whiteAlpha.50" borderColor="whiteAlpha.200" borderWidth="1px">
        <CardHeader>
          <Heading size="md" color="white">
            ROS Gazebo Simulation
          </Heading>
          <Text fontSize="sm" color="gray.400" mt={2}>
            Upload URDF and World files to run a real Gazebo simulation
          </Text>
        </CardHeader>
        
        <CardBody>
          <VStack spacing={6} align="stretch">
            {/* File Upload Section */}
            <Box>
              <Heading size="sm" color="white" mb={4}>
                1. Upload Simulation Files
              </Heading>
              
              <HStack spacing={4}>
                {/* URDF File Upload */}
                <FormControl>
                  <FormLabel color="gray.300" fontSize="sm">
                    URDF File (.urdf, .xacro)
                  </FormLabel>
                  <Input
                    type="file"
                    accept=".urdf,.xacro"
                    onChange={(e) => handleFileChange(e.target.files[0], 'urdf')}
                    bg="blackAlpha.300"
                    borderColor="whiteAlpha.300"
                    color="white"
                    _hover={{ borderColor: 'whiteAlpha.500' }}
                    _focus={{ borderColor: 'blue.400' }}
                  />
                  {urdfFile && (
                    <Text fontSize="xs" color="green.400" mt={1}>
                      ✓ {urdfFile.name}
                    </Text>
                  )}
                </FormControl>

                {/* World File Upload */}
                <FormControl>
                  <FormLabel color="gray.300" fontSize="sm">
                    World File (.world)
                  </FormLabel>
                  <Input
                    type="file"
                    accept=".world"
                    onChange={(e) => handleFileChange(e.target.files[0], 'world')}
                    bg="blackAlpha.300"
                    borderColor="whiteAlpha.300"
                    color="white"
                    _hover={{ borderColor: 'whiteAlpha.500' }}
                    _focus={{ borderColor: 'blue.400' }}
                  />
                  {worldFile && (
                    <Text fontSize="xs" color="green.400" mt={1}>
                      ✓ {worldFile.name}
                    </Text>
                  )}
                </FormControl>
              </HStack>

              <Button
                leftIcon={<FiUpload />}
                onClick={uploadFiles}
                isLoading={uploading}
                loadingText="Uploading..."
                colorScheme="blue"
                mt={4}
                isDisabled={!urdfFile || !worldFile}
              >
                Upload Files
              </Button>

              {uploadResult && (
                <Alert status="success" mt={4} bg="green.900" borderColor="green.500">
                  <AlertIcon />
                  <Box>
                    <Text fontWeight="bold">Files uploaded successfully!</Text>
                    <Text fontSize="sm">
                      URDF: {uploadResult.urdf_filename} | World: {uploadResult.world_filename}
                    </Text>
                    <Text fontSize="xs" color="gray.300">
                      Upload ID: {uploadResult.upload_id}
                    </Text>
                  </Box>
                </Alert>
              )}
            </Box>

            <Divider borderColor="whiteAlpha.300" />

            {/* Simulation Settings */}
            <Box>
              <Heading size="sm" color="white" mb={4}>
                2. Simulation Settings
              </Heading>
              
              <FormControl maxW="200px">
                <FormLabel color="gray.300" fontSize="sm">
                  Duration (seconds)
                </FormLabel>
                <Input
                  type="number"
                  value={duration}
                  onChange={(e) => setDuration(parseInt(e.target.value) || 10)}
                  min={5}
                  max={60}
                  bg="blackAlpha.300"
                  borderColor="whiteAlpha.300"
                  color="white"
                  _hover={{ borderColor: 'whiteAlpha.500' }}
                  _focus={{ borderColor: 'blue.400' }}
                />
              </FormControl>
            </Box>

            <Divider borderColor="whiteAlpha.300" />

            {/* Run Simulation */}
            <Box>
              <Heading size="sm" color="white" mb={4}>
                3. Run Simulation
              </Heading>
              
              <Button
                leftIcon={<FiPlay />}
                onClick={runSimulation}
                isLoading={running}
                loadingText="Running Simulation..."
                colorScheme="green"
                isDisabled={!uploadResult}
                size="lg"
              >
                Run Gazebo Simulation
              </Button>

              {running && (
                <Box mt={4}>
                  <Text color="gray.300" mb={2}>
                    Running simulation... This may take up to {duration + 30} seconds
                  </Text>
                  <Progress isIndeterminate colorScheme="green" />
                  <Text fontSize="xs" color="gray.400" mt={2}>
                    The simulation will start ROS, launch Gazebo, spawn the robot, and record a video.
                  </Text>
                </Box>
              )}

              {simulationResult && (
                <Box mt={4}>
                  {simulationResult.success ? (
                    <Alert status="success" bg="green.900" borderColor="green.500">
                      <AlertIcon />
                      <Box flex="1">
                        <Text fontWeight="bold">Simulation completed successfully!</Text>
                        <Text fontSize="sm">Execution ID: {simulationResult.execution_id}</Text>
                      </Box>
                    </Alert>
                  ) : (
                    <Alert status="error" bg="red.900" borderColor="red.500">
                      <AlertIcon />
                      <Box flex="1">
                        <Text fontWeight="bold">Simulation failed</Text>
                        <Text fontSize="sm">{simulationResult.error}</Text>
                        <HStack mt={2} spacing={2}>
                          <Button size="xs" colorScheme="red" variant="outline" onClick={onOpen}>
                            <FiInfo style={{ marginRight: '4px' }} />
                            View Error Details
                          </Button>
                          {simulationResult.execution_id && (
                            <Button 
                              size="xs" 
                              colorScheme="yellow" 
                              variant="outline"
                              onClick={() => fetchLogs(simulationResult.execution_id)}
                              isLoading={loadingLogs}
                            >
                              <FiEye style={{ marginRight: '4px' }} />
                              View Logs
                            </Button>
                          )}
                        </HStack>
                      </Box>
                    </Alert>
                  )}
                </Box>
              )}
            </Box>

            {/* Simulation Logs */}
            {logs && (
              <>
                <Divider borderColor="whiteAlpha.300" />
                <Box>
                  <Accordion allowToggle>
                    <AccordionItem border="none">
                      <AccordionButton 
                        bg="blackAlpha.300" 
                        _hover={{ bg: "blackAlpha.400" }}
                        borderRadius="md"
                      >
                        <Box flex="1" textAlign="left">
                          <HStack>
                            <Text color="white" fontWeight="bold">
                              Simulation Logs
                            </Text>
                            {loadingLogs && <Spinner size="sm" />}
                          </HStack>
                        </Box>
                        <AccordionIcon color="white" />
                      </AccordionButton>
                      <AccordionPanel pb={4}>
                        <Box
                          bg="black"
                          p={4}
                          borderRadius="md"
                          maxH="400px"
                          overflowY="auto"
                          border="1px solid"
                          borderColor="whiteAlpha.300"
                        >
                          {formatLogOutput(logs)}
                        </Box>
                      </AccordionPanel>
                    </AccordionItem>
                  </Accordion>
                </Box>
              </>
            )}

            {/* Video Results */}
            {simulationResult && simulationResult.success && simulationResult.video_url && (
              <>
                <Divider borderColor="whiteAlpha.300" />
                <Box>
                  <Flex align="center" mb={4}>
                    <Heading size="sm" color="white">
                      4. Simulation Video
                    </Heading>
                    <Spacer />
                    <Button
                      leftIcon={<FiDownload />}
                      onClick={downloadVideo}
                      colorScheme="purple"
                      size="sm"
                    >
                      Download Video
                    </Button>
                  </Flex>

                  <Box 
                    bg="blackAlpha.400" 
                    p={4} 
                    borderRadius="md" 
                    borderColor="whiteAlpha.300" 
                    borderWidth="1px"
                  >
                    <video
                      controls
                      width="100%"
                      style={{ maxHeight: '400px' }}
                      src={`http://localhost:8000${simulationResult.video_url}`}
                    >
                      Your browser does not support the video tag.
                    </video>
                  </Box>
                </Box>
              </>
            )}
          </VStack>
        </CardBody>
      </Card>

      {/* Error Details Dialog */}
      <AlertDialog isOpen={isOpen} onClose={onClose}>
        <AlertDialogOverlay>
          <AlertDialogContent bg="gray.800" color="white" maxW="3xl">
            <AlertDialogHeader fontSize="lg" fontWeight="bold">
              Simulation Error Details
            </AlertDialogHeader>

            <AlertDialogBody>
              {simulationError && (
                <VStack spacing={4} align="stretch">
                  <Box>
                    <Text fontWeight="bold" color="red.300" mb={2}>Error Message:</Text>
                    <Code p={3} display="block" whiteSpace="pre-wrap" bg="red.900" color="red.100">
                      {simulationError.message}
                    </Code>
                  </Box>
                  
                  {simulationError.executionId && (
                    <Box>
                      <Text fontWeight="bold" color="blue.300" mb={2}>Execution ID:</Text>
                      <Code p={2} bg="blue.900" color="blue.100">
                        {simulationError.executionId}
                      </Code>
                    </Box>
                  )}

                  {simulationError.networkError && (
                    <Box>
                      <Text fontWeight="bold" color="orange.300" mb={2}>Troubleshooting Steps:</Text>
                      <VStack align="start" spacing={1} fontSize="sm" color="gray.300">
                        <Text>1. Check if the backend server is running on http://localhost:8000</Text>
                        <Text>2. Verify Docker is installed and running</Text>
                        <Text>3. Ensure the robot-simulation Docker image is built</Text>
                        <Text>4. Check your internet connection</Text>
                        <Text>5. Look at the browser console for more details</Text>
                      </VStack>
                    </Box>
                  )}

                  {!simulationError.networkError && (
                    <Box>
                      <Text fontWeight="bold" color="yellow.300" mb={2}>Common Solutions:</Text>
                      <VStack align="start" spacing={1} fontSize="sm" color="gray.300">
                        <Text>1. Verify URDF and World files are valid and properly formatted</Text>
                        <Text>2. Check if the robot model is compatible with Gazebo</Text>
                        <Text>3. Ensure sufficient system resources (RAM, CPU)</Text>
                        <Text>4. Try reducing simulation duration if timeout occurred</Text>
                        <Text>5. Check the simulation logs for more specific error information</Text>
                      </VStack>
                    </Box>
                  )}
                </VStack>
              )}
            </AlertDialogBody>

            <AlertDialogFooter>
              <Button colorScheme="blue" onClick={onClose}>
                Close
              </Button>
            </AlertDialogFooter>
          </AlertDialogContent>
        </AlertDialogOverlay>
      </AlertDialog>
    </Box>
  );
};

export default SimulationUploader;