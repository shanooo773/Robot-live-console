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
import { motion } from "framer-motion";

// Motion components
const MotionBox = motion(Box);
const MotionCard = motion(Card);
const MotionVStack = motion(VStack);

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
        <MotionVStack 
          spacing={6} 
          textAlign="center"
          initial={{ opacity: 0, y: 50 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8, ease: "easeOut" }}
        >
          <MotionBox
            fontSize="6xl"
            initial={{ scale: 0 }}
            animate={{ scale: 1 }}
            transition={{ duration: 0.5, delay: 0.2, type: "spring", stiffness: 260, damping: 20 }}
          >
            🤖
          </MotionBox>
          <MotionBox
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 0.4 }}
          >
            <Text 
              fontSize="5xl" 
              fontWeight="800" 
              bgGradient="linear(to-r, #667eea, #764ba2, #f093fb)"
              bgClip="text"
              lineHeight="1.2"
            >
              Robot Programming Console
            </Text>
          </MotionBox>
          <MotionBox
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 0.6 }}
          >
            <Text fontSize="xl" color="gray.300" maxW="2xl" lineHeight="1.6">
              Book your time slot to program real robots in simulation. 
              Write Python code, control robots, and see live results.
            </Text>
          </MotionBox>
          <MotionBox
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 0.8 }}
          >
            <Button
              size="lg"
              variant="gradient"
              onClick={onGetStarted}
              fontSize="lg"
              px={12}
              py={8}
              h="auto"
              borderRadius="full"
              _hover={{
                transform: "translateY(-3px)",
                boxShadow: "0 15px 35px rgba(102, 126, 234, 0.4)",
              }}
            >
              Get Started →
            </Button>
          </MotionBox>
        </MotionVStack>

        {/* How It Works */}
        <MotionVStack 
          spacing={8} 
          w="full"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.8, delay: 1 }}
        >
          <MotionBox
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 1.2 }}
          >
            <Text 
              fontSize="4xl" 
              fontWeight="bold" 
              color="white" 
              textAlign="center"
              mb={4}
            >
              How It Works
            </Text>
          </MotionBox>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            {[
              {
                icon: "📅",
                title: "1. Book a Time Slot",
                description: "Choose an available time slot that fits your schedule",
                delay: 1.4
              },
              {
                icon: "💻", 
                title: "2. Write Python Code",
                description: "Use our Monaco editor to write ROS Python code for robots",
                delay: 1.6
              },
              {
                icon: "🎥",
                title: "3. See Results", 
                description: "Watch your robot perform tasks in real-time simulation",
                delay: 1.8
              }
            ].map((step, index) => (
              <MotionCard
                key={index}
                variant="glassmorphism"
                initial={{ opacity: 0, y: 50 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.6, delay: step.delay }}
                whileHover={{ 
                  y: -8,
                  transition: { duration: 0.2 }
                }}
              >
                <CardBody textAlign="center" p={8}>
                  <MotionBox
                    fontSize="4xl" 
                    mb={4}
                    initial={{ scale: 0 }}
                    animate={{ scale: 1 }}
                    transition={{ duration: 0.4, delay: step.delay + 0.2, type: "spring" }}
                  >
                    {step.icon}
                  </MotionBox>
                  <Text fontSize="xl" fontWeight="bold" color="white" mb={3}>
                    {step.title}
                  </Text>
                  <Text color="gray.300" lineHeight="1.6">
                    {step.description}
                  </Text>
                </CardBody>
              </MotionCard>
            ))}
          </SimpleGrid>
        </MotionVStack>

        {/* Available Robots */}
        <MotionVStack 
          spacing={8} 
          w="full"
          initial={{ opacity: 0 }}
          animate={{ opacity: 1 }}
          transition={{ duration: 0.8, delay: 2 }}
        >
          <MotionBox
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 2.2 }}
          >
            <Text 
              fontSize="4xl" 
              fontWeight="bold" 
              color="white" 
              textAlign="center"
              mb={4}
            >
              Available Robots
            </Text>
          </MotionBox>
          
          <SimpleGrid columns={{ base: 1, md: 3 }} spacing={8} w="full">
            {robotTypes.map((robot, index) => (
              <MotionCard
                key={robot.id}
                variant="glassmorphism"
                cursor="pointer"
                onClick={() => handleRobotClick(robot)}
                initial={{ opacity: 0, y: 50, scale: 0.9 }}
                animate={{ 
                  opacity: 1, 
                  y: 0, 
                  scale: 1,
                }}
                transition={{ 
                  duration: 0.6, 
                  delay: 2.4 + (index * 0.2),
                  type: "spring",
                  stiffness: 100
                }}
                whileHover={{ 
                  y: -10,
                  scale: 1.02,
                  transition: { duration: 0.2 }
                }}
                whileTap={{ scale: 0.98 }}
              >
                <CardBody textAlign="center" p={8}>
                  <MotionBox
                    fontSize="6xl" 
                    mb={4}
                    initial={{ scale: 0, rotate: -180 }}
                    animate={{ scale: 1, rotate: 0 }}
                    transition={{ 
                      duration: 0.8, 
                      delay: 2.6 + (index * 0.2), 
                      type: "spring",
                      stiffness: 200 
                    }}
                  >
                    {robot.image}
                  </MotionBox>
                  <Text fontSize="xl" fontWeight="bold" color="white" mb={3}>
                    {robot.name}
                  </Text>
                  <Text color="gray.300" mb={4} lineHeight="1.6">
                    {robot.description}
                  </Text>
                  <VStack spacing={2}>
                    {robot.features.map((feature, featIndex) => (
                      <MotionBox
                        key={featIndex}
                        initial={{ opacity: 0, x: -20 }}
                        animate={{ opacity: 1, x: 0 }}
                        transition={{ 
                          duration: 0.4, 
                          delay: 2.8 + (index * 0.2) + (featIndex * 0.1) 
                        }}
                      >
                        <Badge variant="gradient" fontSize="xs">
                          {feature}
                        </Badge>
                      </MotionBox>
                    ))}
                  </VStack>
                </CardBody>
              </MotionCard>
            ))}
          </SimpleGrid>
        </MotionVStack>

        {/* Call to Action */}
        <MotionVStack 
          spacing={6} 
          textAlign="center" 
          py={12}
          initial={{ opacity: 0, scale: 0.8 }}
          animate={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.8, delay: 3.5 }}
        >
          <MotionBox
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 3.7 }}
          >
            <Text 
              fontSize="3xl" 
              fontWeight="bold" 
              bgGradient="linear(to-r, #4facfe, #00f2fe)"
              bgClip="text"
              mb={4}
            >
              Ready to start programming robots?
            </Text>
          </MotionBox>
          <MotionBox
            initial={{ opacity: 0, y: 20 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ duration: 0.6, delay: 3.9 }}
          >
            <Button
              size="xl"
              variant="glassmorphism"
              onClick={onGetStarted}
              fontSize="xl"
              px={16}
              py={8}
              h="auto"
              borderRadius="full"
              _hover={{
                transform: "translateY(-3px)",
                boxShadow: "0 20px 40px rgba(79, 172, 254, 0.3)",
                background: "rgba(255, 255, 255, 0.2)",
              }}
            >
              Book Your Session Now ✨
            </Button>
          </MotionBox>
        </MotionVStack>
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