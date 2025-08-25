import {
  Box,
  VStack,
  HStack,
  Text,
  Button,
  Container,
  Modal,
  ModalOverlay,
  ModalContent,
  ModalHeader,
  ModalBody,
  ModalCloseButton,
  useDisclosure,
  SimpleGrid,
  Card,
  CardBody,
  Image,
  Badge,
} from "@chakra-ui/react";
import { useState } from "react";

const robotTypes = [
  {
    id: "turtlebot",
    name: "TurtleBot3",
    description: "Mobile robot for navigation and path planning",
    image: "🤖",
    features: ["Autonomous Navigation", "SLAM Mapping", "Obstacle Avoidance"]
  },
  {
    id: "arm",
    name: "Robot Arm",
    description: "6-DOF manipulator for pick and place operations",
    image: "🦾",
    features: ["6 Degrees of Freedom", "Precise Positioning", "Object Manipulation"]
  },
  {
    id: "hand",
    name: "Robot Hand",
    description: "Dexterous gripper for complex manipulation tasks",
    image: "🤲",
    features: ["Multi-finger Grasping", "Force Feedback", "Adaptive Grip"]
  }
];

const LandingPage = ({ onGetStarted }) => {
  const { isOpen, onOpen, onClose } = useDisclosure();
  const [selectedRobot, setSelectedRobot] = useState(null);

  const handleRobotClick = (robot) => {
    setSelectedRobot(robot);
    onOpen();
  };

  return (
    <Container maxW="7xl" py={20}>
      <VStack spacing={16} align="center">
        {/* Hero Section */}
        <VStack spacing={6} textAlign="center">
          <Text fontSize="6xl">🤖</Text>
          <Text fontSize="4xl" fontWeight="bold" color="white">
            Robot Programming Console
          </Text>
          <Text fontSize="xl" color="gray.300" maxW="2xl">
            Book your time slot to program real robots in simulation. 
            Write Python code, control robots, and see live results.
          </Text>
          <Button
            size="lg"
            colorScheme="blue"
            onClick={onGetStarted}
            fontSize="lg"
            px={8}
            py={6}
            h="auto"
          >
            Get Started
          </Button>
        </VStack>

        {/* How It Works */}
        <VStack spacing={8} w="full">
          <Text fontSize="3xl" fontWeight="bold" color="white">
            How It Works
          </Text>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            <Card bg="gray.800" border="1px solid" borderColor="gray.600">
              <CardBody textAlign="center">
                <Text fontSize="4xl" mb={4}>📅</Text>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={2}>
                  1. Book a Time Slot
                </Text>
                <Text color="gray.300">
                  Choose an available time slot that fits your schedule
                </Text>
              </CardBody>
            </Card>

            <Card bg="gray.800" border="1px solid" borderColor="gray.600">
              <CardBody textAlign="center">
                <Text fontSize="4xl" mb={4}>💻</Text>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={2}>
                  2. Write Python Code
                </Text>
                <Text color="gray.300">
                  Use our Monaco editor to write ROS Python code for robots
                </Text>
              </CardBody>
            </Card>

            <Card bg="gray.800" border="1px solid" borderColor="gray.600">
              <CardBody textAlign="center">
                <Text fontSize="4xl" mb={4}>🎥</Text>
                <Text fontSize="xl" fontWeight="bold" color="white" mb={2}>
                  3. See Results
                </Text>
                <Text color="gray.300">
                  Watch your robot perform tasks in real-time simulation
                </Text>
              </CardBody>
            </Card>
          </SimpleGrid>
        </VStack>

        {/* Available Robots */}
        <VStack spacing={8} w="full">
          <Text fontSize="3xl" fontWeight="bold" color="white">
            Available Robots
          </Text>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            {robotTypes.map((robot) => (
              <Card 
                key={robot.id} 
                bg="gray.800" 
                border="1px solid" 
                borderColor="gray.600"
                cursor="pointer"
                onClick={() => handleRobotClick(robot)}
                _hover={{ 
                  borderColor: "blue.400", 
                  transform: "translateY(-4px)",
                  transition: "all 0.2s"
                }}
              >
                <CardBody textAlign="center">
                  <Text fontSize="6xl" mb={4}>{robot.image}</Text>
                  <Text fontSize="xl" fontWeight="bold" color="white" mb={2}>
                    {robot.name}
                  </Text>
                  <Text color="gray.300" mb={4}>
                    {robot.description}
                  </Text>
                  <VStack spacing={2}>
                    {robot.features.map((feature, index) => (
                      <Badge key={index} colorScheme="blue" variant="subtle">
                        {feature}
                      </Badge>
                    ))}
                  </VStack>
                </CardBody>
              </Card>
            ))}
          </SimpleGrid>
        </VStack>

        {/* Call to Action */}
        <VStack spacing={6} textAlign="center" py={12}>
          <Text fontSize="2xl" fontWeight="bold" color="white">
            Ready to start programming robots?
          </Text>
          <Button
            size="lg"
            colorScheme="green"
            onClick={onGetStarted}
            fontSize="lg"
            px={12}
            py={6}
            h="auto"
          >
            Book Your Session Now
          </Button>
        </VStack>
      </VStack>

      {/* Robot Details Modal */}
      <Modal isOpen={isOpen} onClose={onClose} size="lg">
        <ModalOverlay />
        <ModalContent bg="gray.800" color="white">
          <ModalHeader>
            <HStack>
              <Text fontSize="3xl">{selectedRobot?.image}</Text>
              <Text>{selectedRobot?.name}</Text>
            </HStack>
          </ModalHeader>
          <ModalCloseButton />
          <ModalBody pb={6}>
            <VStack spacing={4} align="start">
              <Text color="gray.300">{selectedRobot?.description}</Text>
              
              <Box>
                <Text fontWeight="bold" mb={2}>Key Features:</Text>
                <VStack spacing={2} align="start">
                  {selectedRobot?.features.map((feature, index) => (
                    <HStack key={index}>
                      <Text color="green.400">✓</Text>
                      <Text>{feature}</Text>
                    </HStack>
                  ))}
                </VStack>
              </Box>

              <Box w="full" pt={4}>
                <Button
                  colorScheme="blue"
                  onClick={() => {
                    onClose();
                    onGetStarted();
                  }}
                  w="full"
                >
                  Start Programming This Robot
                </Button>
              </Box>
            </VStack>
          </ModalBody>
        </ModalContent>
      </Modal>
    </Container>
  );
};

export default LandingPage;