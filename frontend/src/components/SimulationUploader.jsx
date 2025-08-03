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
  Spacer
} from '@chakra-ui/react';
import { FiUpload, FiPlay, FiDownload } from 'react-icons/fi';

const SimulationUploader = () => {
  const [urdfFile, setUrdfFile] = useState(null);
  const [worldFile, setWorldFile] = useState(null);
  const [duration, setDuration] = useState(10);
  const [uploading, setUploading] = useState(false);
  const [running, setRunning] = useState(false);
  const [uploadResult, setUploadResult] = useState(null);
  const [simulationResult, setSimulationResult] = useState(null);
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
        throw new Error(`Upload failed: ${response.statusText}`);
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

      if (!response.ok) {
        throw new Error(`Simulation failed: ${response.statusText}`);
      }

      const result = await response.json();
      setSimulationResult(result);

      if (result.success) {
        toast({
          title: 'Simulation Completed',
          description: `Video generated successfully`,
          status: 'success',
          duration: 3000,
          isClosable: true,
        });
      } else {
        toast({
          title: 'Simulation Failed',
          description: result.error,
          status: 'error',
          duration: 5000,
          isClosable: true,
        });
      }
    } catch (error) {
      toast({
        title: 'Simulation Error',
        description: error.message,
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
                      <Box>
                        <Text fontWeight="bold">Simulation failed</Text>
                        <Text fontSize="sm">{simulationResult.error}</Text>
                      </Box>
                    </Alert>
                  )}
                </Box>
              )}
            </Box>

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
    </Box>
  );
};

export default SimulationUploader;